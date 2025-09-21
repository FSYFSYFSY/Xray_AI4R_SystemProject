#include <functional>
#include <memory>
#include <thread>
#include <atomic>
#include <cstdio>
#include <string>
#include <cstring>
#include <cstdlib>
// POSIX I/O for graceful non-blocking wait
#include <fcntl.h>
// select() to wait on fd with timeout
#include <sys/select.h>
#include <dirent.h>
#include <unistd.h>
#include <limits.h>

#include "rclcpp/rclcpp.hpp"
#include "ai4r_interfaces/msg/imu.hpp"
//#include "ai4r_interfaces/msg/tof_sensor.hpp"

class SensorPcbParserNode : public rclcpp::Node {
public:
  SensorPcbParserNode() : Node("sensor_pcb_parser_node") {
    // Publishers
    imu_topic_pub_ = this->create_publisher<ai4r_interfaces::msg::Imu>("imu_sensor", rclcpp::QoS(10));
    //tof_topic_pub_ = this->create_publisher<ai4r_interfaces::msg::TofSensor>("tof_sensor_vl53l5cx", rclcpp::QoS(10));

    log_info("[SENSOR PCB PARSE NODE] INFO: Node started");

    // Allow selecting a specific Pico by stable by-path substring
    static constexpr const char* kDefaultByPathSubstr = "usb-0:1.3"; // RP1 default port
    std::string by_path_substr = this->declare_parameter<std::string>("serial_by_path_substr", kDefaultByPathSubstr);
    std::string devpath = "/dev/ttyACM0";
    if (!by_path_substr.empty()) {
      const char *dirpath = "/dev/serial/by-path";
      DIR *dir = opendir(dirpath);
      if (dir) {
        struct dirent *ent;
        while ((ent = readdir(dir)) != nullptr) {
          if (ent->d_name[0] == '.') continue;
          std::string name(ent->d_name);
          if (name.find(by_path_substr) != std::string::npos) {
            std::string linkpath = std::string(dirpath) + "/" + name;
            char buf[PATH_MAX];
            ssize_t n = readlink(linkpath.c_str(), buf, sizeof(buf) - 1);
            if (n > 0) {
              buf[n] = '\0';
              std::string rel(buf);
              if (rel.rfind("/dev/", 0) == 0) {
                devpath = rel;
              } else {
                devpath = std::string("/dev/") + rel.substr(rel.find_last_of('/') + 1);
              }
              break;
            }
          }
        }
        closedir(dir);
      }
    }
    serial_port_ = std::fopen(devpath.c_str(), "r");
    if (!serial_port_) {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", devpath.c_str());
      return;
    }

    running_.store(true);
    reader_thread_ = std::thread([this]() { this->read_loop(); });
  }

  ~SensorPcbParserNode() override {
    log_info("[SENSOR PCB PARSE NODE] INFO: Now joining the sensor reading thread");
    running_.store(false);
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
    log_info("[SENSOR PCB PARSE NODE] INFO: Now closing the serial port (press Ctrl+C again if it stalls)");
    if (serial_port_) {
      std::fclose(serial_port_);
      serial_port_ = nullptr;
    }
    log_info("[SENSOR PCB PARSE NODE] INFO: Now shutting down the node");
  }

private:
  void read_loop() {
    if (!serial_port_) {
      return;
    }

    // Messages are ASCII encoded from the pico board as follows:
    // IMU G/A/M ...
    // TOF tof_num usec distance_mm_1 ... distance_mm_16

    constexpr size_t BUF_SZ = 256;
    char read_buf[BUF_SZ];
    std::memset(&read_buf, '\0', sizeof(read_buf));

    const int fd = fileno(serial_port_);
    if (fd < 0) {
      RCLCPP_ERROR(get_logger(), "Invalid file descriptor for serial port");
      return;
    }

    while (rclcpp::ok() && running_.load()) {
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(fd, &rfds);
      timeval tv{};
      tv.tv_sec = 0;
      tv.tv_usec = 200000; // 200ms

      int rv = ::select(fd + 1, &rfds, nullptr, nullptr, &tv);
      if (rv > 0 && FD_ISSET(fd, &rfds)) {
        if (std::fgets(read_buf, sizeof(read_buf), serial_port_) != nullptr) {
          if (read_buf[0] == '\n' || read_buf[1] == '\0') {
            continue;
          }

          char *token = std::strtok(read_buf, " ");
          if (!token) {
            continue;
          }

          if (std::strcmp(token, "IMU\0") == 0 || std::strcmp(token, "IMU") == 0) {
            token = std::strtok(nullptr, " ");
            if (!token) {
              continue;
            }

            // Preserve behavior: publish the IMU message after any of G/A/M updates
            if (std::strcmp(token, "G\0") == 0 || std::strcmp(token, "G") == 0) {
              imu_msg_.usec_since_last_gyro_msg = std::atoll(std::strtok(nullptr, " "));
              imu_msg_.yaw = std::atof(std::strtok(nullptr, " "));
              imu_msg_.pitch = std::atof(std::strtok(nullptr, " "));
              imu_msg_.roll = std::atof(std::strtok(nullptr, " "));
            } else if (std::strcmp(token, "A\0") == 0 || std::strcmp(token, "A") == 0) {
              imu_msg_.usec_since_last_accel_msg = std::atoll(std::strtok(nullptr, " "));
              imu_msg_.accelx = std::atof(std::strtok(nullptr, " "));
              imu_msg_.accely = std::atof(std::strtok(nullptr, " "));
              imu_msg_.accelz = std::atof(std::strtok(nullptr, " "));
            } else if (std::strcmp(token, "M\0") == 0 || std::strcmp(token, "M") == 0) {
              imu_msg_.usec_since_last_mag_msg = std::atoll(std::strtok(nullptr, " "));
              imu_msg_.magx = std::atof(std::strtok(nullptr, " "));
              imu_msg_.magy = std::atof(std::strtok(nullptr, " "));
              imu_msg_.magz = std::atof(std::strtok(nullptr, " "));
              char *acc_tok = std::strtok(nullptr, " ");
              if (acc_tok) {
                imu_msg_.mag_accuracy = std::atoi(acc_tok);
              }
            }
            imu_topic_pub_->publish(imu_msg_);
          }
          // else if (std::strcmp(token, "TOF\0") == 0 || std::strcmp(token, "TOF") == 0) {
          //   char *tof_tok = std::strtok(nullptr, " ");
          //   if (!tof_tok) {
          //     continue;
          //   }
          //   int tof_num = std::atoi(tof_tok);
          //   ai4r_interfaces::msg::TofSensor msg;
          //   msg.usec_since_last_msg = std::atoll(std::strtok(nullptr, " "));
          //   for (int i = 0; i < 16; i++) {
          //     char *d = std::strtok(nullptr, " ");
          //     if (!d) {
          //       break;
          //     }
          //     msg.distance_mm[i] = std::atoi(d);
          //   }
          //   msg.sensor_id = static_cast<uint16_t>(tof_num);
          //   tof_topic_pub_->publish(msg);
          // }
        } else {
          // EOF or error
          break;
        }
      } else if (rv < 0) {
        // select error
        break;
      }
      // rv == 0 -> timeout; loop again to check running_
    }
  }

  // Publishers
  rclcpp::Publisher<ai4r_interfaces::msg::Imu>::SharedPtr imu_topic_pub_;
  //rclcpp::Publisher<ai4r_interfaces::msg::TofSensor>::SharedPtr tof_topic_pub_;

  // State
  std::thread reader_thread_{};
  std::atomic<bool> running_{false};
  std::FILE *serial_port_{nullptr};
  ai4r_interfaces::msg::Imu imu_msg_{}; // Persist across IMU sub-messages

  void log_info(const char* msg) {
    if (rclcpp::ok()) {
      RCLCPP_INFO(get_logger(), "%s", msg);
    } else {
      std::printf("%s\n", msg);
      std::fflush(stdout);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorPcbParserNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
