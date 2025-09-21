#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ai4r_interfaces/msg/esc_and_steering_pulse_width.hpp>

#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <atomic>
#include <dirent.h>
#include <unistd.h>

namespace {
std::string find_serial_device_by_path_substr(const std::string &substr) {
  const char *dirpath = "/dev/serial/by-path";
  DIR *dir = opendir(dirpath);
  if (!dir) return {};
  struct dirent *ent;
  while ((ent = readdir(dir)) != nullptr) {
    if (ent->d_name[0] == '.') continue;
    std::string name(ent->d_name);
    if (name.find(substr) != std::string::npos) {
      std::string linkpath = std::string(dirpath) + "/" + name;
      char buf[PATH_MAX];
      ssize_t n = readlink(linkpath.c_str(), buf, sizeof(buf) - 1);
      if (n > 0) {
        buf[n] = '\0';
        std::string rel(buf);
        // readlink gives a relative path like ../../ttyACM0
        // Normalize to absolute device path
        std::string devpath;
        if (rel.rfind("../", 0) == 0) {
          devpath = std::string("/dev/") + rel.substr(rel.find_last_of('/') + 1);
        } else if (rel.rfind("/dev/", 0) == 0) {
          devpath = rel;
        } else {
          devpath = std::string("/dev/") + rel;
        }
        closedir(dir);
        return devpath;
      }
    }
  }
  closedir(dir);
  return {};
}
}

class Rp2BridgeNode : public rclcpp::Node {
public:
  Rp2BridgeNode() : Node("rp2_bridge_node") {
    // Debug logging toggle for RX/TX payloads
    constexpr bool BRIDGE_DEBUG_LOG = false;
    static constexpr const char* kDefaultByPathSubstr = "usb-0:1.4"; // RP2 default port
    serial_by_path_substr_ = this->declare_parameter<std::string>("serial_by_path_substr", kDefaultByPathSubstr);
    esc_steer_sub_ = this->create_subscription<ai4r_interfaces::msg::EscAndSteeringPulseWidth>(
        "rp2/esc_and_steering", 10,
        std::bind(&Rp2BridgeNode::on_esc_steer, this, std::placeholders::_1));
    test_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "rp2/test_mode", 10,
        std::bind(&Rp2BridgeNode::on_test, this, std::placeholders::_1));

    speed_period_pub_ = this->create_publisher<std_msgs::msg::UInt32>("rp2/speed_period_us", 10);
    speed_duty_pub_ = this->create_publisher<std_msgs::msg::Float32>("rp2/speed_duty", 10);
    mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("rp2/mode", 10);

    open_serial();
    running_.store(true);
    reader_ = std::thread([this]() { this->read_loop(); });
  }

  ~Rp2BridgeNode() override {
    running_.store(false);
    if (reader_.joinable()) reader_.join();
    if (fp_in_) { std::fclose(fp_in_); fp_in_ = nullptr; }
    if (fp_out_) { std::fclose(fp_out_); fp_out_ = nullptr; }
  }

private:
  void open_serial() {
    std::string dev;
    if (!serial_by_path_substr_.empty()) {
      dev = find_serial_device_by_path_substr(serial_by_path_substr_);
    }
    if (dev.empty()) {
      dev = "/dev/ttyACM0";
    }
    // Open separate streams for read and write to avoid stdio FILE lock contention
    fp_in_ = std::fopen(dev.c_str(), "r");
    if (!fp_in_) {
      RCLCPP_ERROR(get_logger(), "Failed to open serial device for read: %s", dev.c_str());
      return;
    }
    fp_out_ = std::fopen(dev.c_str(), "w");
    if (!fp_out_) {
      RCLCPP_ERROR(get_logger(), "Failed to open serial device for write: %s", dev.c_str());
      std::fclose(fp_in_);
      fp_in_ = nullptr;
      return;
    }
    // Unbuffered write to reduce latency
    std::setvbuf(fp_out_, nullptr, _IONBF, 0);
    RCLCPP_INFO(get_logger(), "Opened serial device: %s", dev.c_str());
  }

  void on_esc_steer(const ai4r_interfaces::msg::EscAndSteeringPulseWidth::SharedPtr msg) {
    drive_us_ = msg->esc_pulse_width;
    steer_us_ = msg->steering_pulse_width;
    // Debug: log received command
    // RCLCPP_INFO(get_logger(), "rx esc_and_steering: esc=%u steer=%u", (unsigned)drive_us_, (unsigned)steer_us_);
    send_cmd();
  }
  void on_test(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!fp_out_) return;
    std::fprintf(fp_out_, "RP2 TEST %s\n", msg->data ? "ON" : "OFF");
    std::fflush(fp_out_);
  }

  void send_cmd() {
    if (!fp_out_) return;
    // Debug: log exactly what we are about to write
    // RCLCPP_INFO(get_logger(), "tx serial: RP2 CMD %u %u", (unsigned)drive_us_, (unsigned)steer_us_);
    std::fprintf(fp_out_, "RP2 CMD %u %u\n", (unsigned)drive_us_, (unsigned)steer_us_);
    std::fflush(fp_out_);
  }

  void read_loop() {
    if (!fp_in_) return;
    char buf[256];
    while (rclcpp::ok() && running_.load()) {
      if (!std::fgets(buf, sizeof(buf), fp_in_)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      if (buf[0] == '\n' || buf[0] == '\0') continue;
      char *tok = std::strtok(buf, " \r\n");
      if (!tok) continue;
      if (std::strcmp(tok, "RP2") == 0) {
        char *sub = std::strtok(nullptr, " \r\n");
        if (!sub) continue;
        if (std::strcmp(sub, "SPD") == 0) {
          char *p = std::strtok(nullptr, " \r\n");
          char *d = std::strtok(nullptr, " \r\n");
          char *m = std::strtok(nullptr, " \r\n");
          if (p && d && m) {
            std_msgs::msg::UInt32 pmsg; pmsg.data = static_cast<uint32_t>(std::strtoul(p, nullptr, 10));
            std_msgs::msg::Float32 dmsg; dmsg.data = std::strtof(d, nullptr);
            std_msgs::msg::Bool mmsg; mmsg.data = (std::atoi(m) != 0);
            speed_period_pub_->publish(pmsg);
            speed_duty_pub_->publish(dmsg);
            mode_pub_->publish(mmsg);
          }
        }
      }
    }
  }

  // Params
  std::string serial_by_path_substr_;

  // Publishers/subscribers
  rclcpp::Subscription<ai4r_interfaces::msg::EscAndSteeringPulseWidth>::SharedPtr esc_steer_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr test_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr speed_period_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_duty_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_pub_;

  // State
  std::FILE *fp_in_ = nullptr;
  std::FILE *fp_out_ = nullptr;
  std::thread reader_{};
  std::atomic<bool> running_{false};
  uint16_t drive_us_ = 1500;
  uint16_t steer_us_ = 1500;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Rp2BridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
