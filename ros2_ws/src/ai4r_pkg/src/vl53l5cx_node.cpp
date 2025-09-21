#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/qos.hpp"

#include "ai4r_interfaces/msg/tof_sensor.hpp"

#include "i2c_driver/i2c_driver.h"
#include "vl53l5cx/vl53l5cx.h"

using namespace std::chrono_literals;

class Vl53l5cxNode : public rclcpp::Node
{
public:
    Vl53l5cxNode()
    : Node("vl53l5cx_node"),
      last_publish_time_(std::chrono::steady_clock::now())
    {
        i2c_device_ = this->declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
        sensor_address_ = static_cast<uint8_t>(this->declare_parameter<int>("sensor_address", 0x29));
        sensor_id_ = static_cast<uint16_t>(this->declare_parameter<int>("sensor_id", 0));
        poll_interval_ms_ = this->declare_parameter<int>("poll_interval_ms", 50);
        resolution_ = static_cast<uint8_t>(this->declare_parameter<int>("resolution", VL53L5CX_RESOLUTION_4X4));
        ranging_frequency_hz_ = static_cast<uint8_t>(this->declare_parameter<int>("ranging_frequency_hz", 20));
        integration_time_ms_ = static_cast<uint8_t>(this->declare_parameter<int>("integration_time_ms", 50));
        sharpener_percent_ = static_cast<uint8_t>(this->declare_parameter<int>("sharpener_percent", 20));
        target_order_ = static_cast<uint8_t>(this->declare_parameter<int>(
            "target_order", VL53L5CX_TARGET_ORDER_CLOSEST));
        ranging_mode_ = static_cast<uint8_t>(this->declare_parameter<int>(
            "ranging_mode", VL53L5CX_RANGING_MODE_CONTINUOUS));

        if (resolution_ != VL53L5CX_RESOLUTION_4X4) {
            RCLCPP_WARN(this->get_logger(),
                        "resolution %u not supported by current output message; forcing 4x4", resolution_);
            resolution_ = VL53L5CX_RESOLUTION_4X4;
        }

        if (poll_interval_ms_ <= 0) {
            RCLCPP_WARN(this->get_logger(), "poll_interval_ms must be positive; defaulting to 100 ms");
            poll_interval_ms_ = 100;
        }

        num_zones_ = static_cast<size_t>(resolution_);

        i2c_driver_ = std::make_unique<I2C_Driver>(i2c_device_.c_str());
        RCLCPP_INFO(this->get_logger(), "Opening I2C device %s", i2c_driver_->get_device_name());
        if (!i2c_driver_->open_i2c_device()) {
            throw std::runtime_error("Failed to open I2C device " + i2c_device_);
        }

        vl53l5cx_ = std::make_unique<VL53L5CX>(i2c_driver_.get(), sensor_address_);

        bool init_success = vl53l5cx_->initialise_and_start_ranging(
            resolution_,
            ranging_frequency_hz_,
            integration_time_ms_,
            sharpener_percent_,
            target_order_,
            ranging_mode_);
        if (!init_success) {
            i2c_driver_->close_i2c_device();
            throw std::runtime_error("Failed to initialise and start VL53L5CX ranging");
        }

        //publisher_ = this->create_publisher<ai4r_interfaces::msg::TofSensor>("tof_sensor", rclcpp::SensorDataQoS());
        publisher_ = this->create_publisher<ai4r_interfaces::msg::TofSensor>("tof_sensor_vl53l5cx", rclcpp::QoS(10));

        poll_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(poll_interval_ms_),
            std::bind(&Vl53l5cxNode::poll_sensor, this));
    }

    ~Vl53l5cxNode() override
    {
        if (vl53l5cx_) {
            vl53l5cx_->stop_ranging();
        }

        if (i2c_driver_) {
            if (!i2c_driver_->close_i2c_device()) {
                RCLCPP_WARN(this->get_logger(), "Failed to close I2C device %s", i2c_device_.c_str());
            }
        }
    }

private:
    void poll_sensor()
    {
        VL53L5CX_ResultsData results{};
        if (!vl53l5cx_->get_ranging_data(&results)) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "No new VL53L5CX data available yet");
            return;
        }

        auto now = std::chrono::steady_clock::now();
        uint64_t usec_since_last = 0U;
        if (has_published_) {
            usec_since_last = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(now - last_publish_time_).count());
        }

        ai4r_interfaces::msg::TofSensor msg;
        msg.usec_since_last_msg = usec_since_last;
        msg.sensor_id = sensor_id_;
        msg.silicon_temp_degc = static_cast<float>(results.silicon_temp_degc);

        msg.distance_mm.assign(results.distance_mm, results.distance_mm + num_zones_);
        msg.target_status.assign(results.target_status, results.target_status + num_zones_);

        publisher_->publish(msg);

        last_publish_time_ = now;
        has_published_ = true;
    }

    std::string i2c_device_;
    uint8_t sensor_address_{};
    uint16_t sensor_id_{};
    int poll_interval_ms_{};
    uint8_t resolution_{};
    uint8_t ranging_frequency_hz_{};
    uint8_t integration_time_ms_{};
    uint8_t sharpener_percent_{};
    uint8_t target_order_{};
    uint8_t ranging_mode_{};
    size_t num_zones_{};

    std::unique_ptr<I2C_Driver> i2c_driver_;
    std::unique_ptr<VL53L5CX> vl53l5cx_;

    rclcpp::Publisher<ai4r_interfaces::msg::TofSensor>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr poll_timer_;

    std::chrono::steady_clock::time_point last_publish_time_;
    bool has_published_ = false;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<Vl53l5cxNode>();
        rclcpp::spin(node);
    } catch (const std::exception & ex) {
        RCLCPP_FATAL(rclcpp::get_logger("vl53l5cx_node"), "Exception: %s", ex.what());
    }
    rclcpp::shutdown();
    return 0;
}



