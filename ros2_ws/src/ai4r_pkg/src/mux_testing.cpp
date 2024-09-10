
#include "ai4r_pkg/traxxas_old_node.hpp"

// Static Global Variables


class TraxxasNode : public rclcpp::Node {
    public:
        TraxxasNode() : Node("traxxas_node") {
            servo_pulse_width_sub_ = this->create_subscription<ai4r_interfaces::msg::ServoPulseWidth>(
                "servo_pulse_width", rclcpp::QoS(10), std::bind(&TraxxasNode::servoSubscriberCallback, this, std::placeholders::_1)
            );

            mux_select_slave_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "mux_select_slave", rclcpp::QoS(10), std::bind(&TraxxasNode::muxSelectSlaveSubscriberCallback, this, std::placeholders::_1)
            );

            // Publisher for the current i2c commands
            current_esc_pulse_width_publisher_ = this->create_publisher<ai4r_interfaces::msg::ServoPulseWidth>("traxxas_esc_current_pulse_width", 10);
            current_steering_pulse_width_publisher_ = this->create_publisher<ai4r_interfaces::msg::ServoPulseWidth>("traxxas_steering_current_pulse_width", 10);

            current_mux_select_pulse_width_publisher_ = this->create_publisher<ai4r_interfaces::msg::ServoPulseWidth>("current_mux_select_pulse_width", 10);

            // Open the I2C device
            // > Note that the I2C driver is already instantiated
            //   as a member variable of this node
            bool open_success = m_i2c_driver.open_i2c_device();

            // Display the status
            if (!open_success) {
                RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] FAILED to open I2C device named " << m_i2c_driver.get_device_name());
            } else {
                RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] Successfully opened named " << m_i2c_driver.get_device_name() << ", with file descriptor = " << m_i2c_driver.get_file_descriptor());
            }

            // SET THE CONFIGURATION OF THE SERVO DRIVER

            // Specify the frequency of the servo driver -> default for this is 200 Hz
            float new_frequency_in_hz = SERVO_FREQUENCY;
            bool verbose_display_for_servo_driver_init = true;
            
            // Call the Servo Driver initialisation function
            bool result_servo_init = m_pca9685_servo_driver.initialise_with_frequency_in_hz(new_frequency_in_hz, verbose_display_for_servo_driver_init);
            
            // Report Servo Board Initialisation Result
            if (!result_servo_init)	{
                RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] FAILED - while initialising servo driver with I2C address " << static_cast<int>(m_pca9685_servo_driver.get_i2c_address()) );
            } else {
                RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] SUCCESS - while initialising servo driver with I2C address " << static_cast<int>(m_pca9685_servo_driver.get_i2c_address()) );
            }
        }

    private:

        // Send a pulse width on the specified channel.
        void setPWMSignal(uint16_t channel, uint16_t pulse_width_in_us) {
            // Call the function to set the desired pulse width
            bool result = m_pca9685_servo_driver.set_pwm_pulse_in_microseconds(channel, pulse_width_in_us);

            // Display if an error occurred
            if (!result) {
                RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] FAILED to set pulse width for servo at channel " << static_cast<int>(channel) );
            }
            else{
                // Publish the value set
                if (channel == STEERING_SERVO_CHANNEL)
                {
                    auto message = ai4r_interfaces::msg::ServoPulseWidth();
                    message.channel = channel;
                    message.pulse_width_in_microseconds = pulse_width_in_us;
                    current_steering_pulse_width_publisher_->publish(message);
                }
                else if (channel == ESC_SERVO_CHANNEL)
                {
                    auto message = ai4r_interfaces::msg::ServoPulseWidth();
                    message.channel = channel;
                    message.pulse_width_in_microseconds = pulse_width_in_us;
                    current_esc_pulse_width_publisher_->publish(message);
                }
                else if (channel == MUX_SELECT_CHANNEL)
                {
                    auto message = ai4r_interfaces::msg::ServoPulseWidth();
                    message.channel = channel;
                    message.pulse_width_in_microseconds = pulse_width_in_us;
                    current_mux_select_pulse_width_publisher_ ->publish(message);
                }
                // else {
                //     RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] Not a registered channel " << static_cast<int>(channel) );
                // }
            }
        }

        // For receiving boolean value to activate the switch (select slave) or not
        void muxSelectSlaveSubscriberCallback(const std_msgs::msg::Bool & msg) {
            // Extract the boolean
            bool select_slave = msg.data;

            // Display the message received
            //RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] Message received for servo with channel = " << static_cast<int>(channel) << ", and pulse width [us] = " << static_cast<int>(pulse_width_in_us) );

            uint16_t pulse_width_in_us;
            if (select_slave == true) {
                pulse_width_in_us = MUX_SELECT_SLAVE_PULSE_WIDTH;
            }
            else {
                pulse_width_in_us = MUX_SELECT_MASTER_PULSE_WIDTH;
            }

            // Set servo PWM signal to this value
            setPWMSignal(MUX_SELECT_CHANNEL, pulse_width_in_us);
        }

        // For receiving PWM values to control the steering (channel 0) or esc (channel 1)
        void servoSubscriberCallback(const ai4r_interfaces::msg::ServoPulseWidth & msg) {
            // Extract the channel and pulse width from the message
            uint16_t channel = msg.channel;
            uint16_t pulse_width_in_us = msg.pulse_width_in_microseconds;

            // Display the message received
            //RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] Message received for servo with channel = " << static_cast<int>(channel) << ", and pulse width [us] = " << static_cast<int>(pulse_width_in_us) );

            // Set servo PWM signal to this value
            setPWMSignal(channel, pulse_width_in_us);
        }

        rclcpp::Subscription<ai4r_interfaces::msg::ServoPulseWidth>::SharedPtr servo_pulse_width_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mux_select_slave_sub_;
        rclcpp::Publisher<ai4r_interfaces::msg::ServoPulseWidth>::SharedPtr current_esc_pulse_width_publisher_;
        rclcpp::Publisher<ai4r_interfaces::msg::ServoPulseWidth>::SharedPtr current_steering_pulse_width_publisher_;
        rclcpp::Publisher<ai4r_interfaces::msg::ServoPulseWidth>::SharedPtr current_mux_select_pulse_width_publisher_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TraxxasNode>();
    rclcpp::spin(node); // Use rclcpp::spin() to handle callbacks.

	// Close the I2C device
	bool close_success = m_i2c_driver.close_i2c_device();
	
	// Display the status
	if (!close_success) {
		RCLCPP_INFO_STREAM(node->get_logger(), "[TRAXXAS] FAILED to close I2C device named " << m_i2c_driver.get_device_name());
	} else {
		RCLCPP_INFO_STREAM(node->get_logger(), "[TRAXXAS] Successfully closed device named " << m_i2c_driver.get_device_name());
	}
    
    rclcpp::shutdown();
    return 0;
}
