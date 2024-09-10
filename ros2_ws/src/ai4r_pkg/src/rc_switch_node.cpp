// First we import the rclcpp library. From rclcpp we’ll be able to retrieve many of the ROS2 core functionalities: nodes, topics, services, etc.
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <gpiod.h>


// Specify the chip name of the GPIO interface
const char * gpio_chip_name = "/dev/gpiochip0";
// Initialise a GPIO chip
struct gpiod_chip *chip;
// GPIO line number to monitor 'GOOD' RC signal (valid or invalid signal)
int good_line_number = 27;
// GPIO line number to monitor 'OUT' RC signal (switch active or inactive)
int out_line_number = 17;


// Create a class which inherits from rclcpp::Node which contains almost all ROS2 basic functionalities. 
// Any addition/change you’ll make to your node will be inside the MyNode class. The main function will stay the same.
class MyNode : public rclcpp::Node
{
public:
    // Constructor is a good place to initiate your node with a name and options. You will also define your publishers/subscribers/services here. The node’s name is now set in the constructor, but nothing prevents you from passing the name from the main if you want, and then use it.
    MyNode() : Node("rc_switch_node") 
    {
        // Publishers for the 'GOOD' (invalid or valid) and 'OUT' (inactive=manual or active=autonomous) RC signals
        good_signal_publisher_ = this->create_publisher<std_msgs::msg::Int8>("rc_signal_validity", 10);
        out_signal_publisher_ = this->create_publisher<std_msgs::msg::Int8>("operating_mode", 10);
        // Publisher to send disable or enable request
        disable_or_enable_request_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("request", 10);

        // Initialize the timer with another inherited method: create_wall_timer(). We need to give 2 arguments: the duration between 2 callbacks, and the function to call. Here to pass the class method we have to use std::bind(). The callback will start being triggered when the node starts spinning.
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&MyNode::timerCallback, this));
    } 

private:
    // Private variables
    int good_val;   // Value of the 'GOOD' signal
    int out_val;    // Value of the 'OUT' signal
    int prev_out_val = -1;   // Previous value of the 'OUT' signal. Initially the previous value is unknown.

    // Add a callback for this timer. We make this method private since it will only be called from within the node class.
    void timerCallback()
    {
        // Get the current value of the lines
        // > Note: the third argument to "gpiod_ctxless_get_value"
        //   is an "active_low" boolean input argument.
        //   If true, this indicate to the function that active state
        //   of this line is low.
        good_val = gpiod_ctxless_get_value(gpio_chip_name, good_line_number, false, "foobar");
        out_val = gpiod_ctxless_get_value(gpio_chip_name, out_line_number, false, "foobar");

        // // Log the status of the good line for debugging
        // RCLCPP_INFO(this->get_logger(), "GOOD line value: '%d'\n", good_val);
        // if (good_val == 0) {
        //     RCLCPP_INFO(this->get_logger(), "GOOD signal is INVALID\n");
        // } else if (good_val == 1) {
        //     RCLCPP_INFO(this->get_logger(), "GOOD signal is VALID\n");
        // } else {
        //     RCLCPP_INFO(this->get_logger(), "GOOD signal is in an UNKNOWN state. Failed to read\n");
        // }
        // // Log the status of the out line for debugging
        // RCLCPP_INFO(this->get_logger(), "OUT line value: '%d'\n", out_val);
        // if (out_val == 0) {
        //     RCLCPP_INFO(this->get_logger(), "OUT signal is INACTIVE. Remote is in MANUAL operating mode.\n");
        // } else if (out_val == 1) {
        //     RCLCPP_INFO(this->get_logger(), "OUT signal is ACTIVE. Remote is in AUTONOMOUS operating mode.\n");
        // } else {
        //     RCLCPP_INFO(this->get_logger(), "OUT signal is in an UNKNOWN state. Failed to read\n");
        // }
            
        // Publish the RC signal validity
        auto good_message = std_msgs::msg::Int8();
        good_message.data = good_val;
        good_signal_publisher_->publish(good_message);
        // Publish the operating mode
        auto out_message = std_msgs::msg::Int8();
        out_message.data = out_val;
        out_signal_publisher_->publish(out_message);

        // Only publish disable or enable request if the value of the 'OUT' signal has changed
        if (prev_out_val != out_val) {
            if (out_val == 0) {
                // Publish the disable message if MANUAL mode is selected
                auto disable_message = std_msgs::msg::UInt8();
                disable_message.data = 0;
                disable_or_enable_request_publisher_->publish(disable_message);
            } else if (out_val == 1) {
                // // Publish the enable message if AUTONOMOUS mode is selected
                // auto enable_message = std_msgs::msg::UInt8();
                // enable_message.data = 1;
                // disable_or_enable_request_publisher_->publish(enable_message);
            }
        }
        // Update the previous value of the 'OUT' signal
        prev_out_val = out_val;
        
    }

    rclcpp::TimerBase::SharedPtr timer_;    // Declare a ROS2 Timer object as a private attribute of the class
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr good_signal_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr out_signal_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr disable_or_enable_request_publisher_;
};

int main(int argc, char **argv)
{

    // Open GPIO chip
    chip = gpiod_chip_open(gpio_chip_name);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>(); // Create an object of the custom class we wrote
    rclcpp::spin(node);

    // Release chip
    gpiod_chip_close(chip);

    rclcpp::shutdown();

    return 0;
}