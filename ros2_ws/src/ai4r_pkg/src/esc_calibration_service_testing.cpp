#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ai4r_pkg/traxxas_old_node.hpp"


// Create a class which inherits from rclcpp::Node which contains almost all ROS2 basic functionalities. 
// Any addition/change you’ll make to your node will be inside the MyNode class. The main function will stay the same.
class MyNode : public rclcpp::Node
{
public:
    // Constructor is a good place to initiate your node with a name and options. You will also define your publishers/subscribers/services here. The node’s name is now set in the constructor, but nothing prevents you from passing the name from the main if you want, and then use it.
    MyNode() : Node("esc_calibration_service_node") 
    {
        // Create a service for the ESC calibration
        esc_calibration_service_ = this->create_service<std_srvs::srv::Trigger>("esc_calibration", std::bind(&MyNode::escCalibrationCallback, this, std::placeholders::_1, std::placeholders::_2));



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
    // Add a callback for this timer. We make this method private since it will only be called from within the node class.
    void escCalibrationCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "ESC calibration service called");
        
        // Perform ESC calibration
        // Full forward
        RCLCPP_INFO(this->get_logger(), "Calibrating ESC: Full forward for 2 seconds");
        setPWMSignal(ESC_SERVO_CHANNEL, ESC_MAXIMUM_PULSE_WIDTH);
        // Wait for 2 seconds
        rclcpp::sleep_for(2s);
        // Full reverse
        RCLCPP_INFO(this->get_logger(), "Calibrating ESC: Full reverse for 2 seconds");
        setPWMSignal(ESC_SERVO_CHANNEL, ESC_MINIMUM_PULSE_WIDTH);
        // Wait for 2 seconds
        rclcpp::sleep_for(2s);
        // Neutral
        RCLCPP_INFO(this->get_logger(), "Calibrating ESC: Neutral");
        setPWMSignal(ESC_SERVO_CHANNEL, ESC_NEUTRAL_PULSE_WIDTH);

        // Service response
        RCLCPP_INFO(this->get_logger(), "ESC calibration complete");
        response->success = true;
        response->message = "ESC calibration complete";
    }




    // Send a pulse width on the specified channel.
    void setPWMSignal(uint16_t channel, uint16_t pulse_width_in_us) {
        // Call the function to set the desired pulse width
        bool result = m_pca9685_servo_driver.set_pwm_pulse_in_microseconds(channel, pulse_width_in_us);

        // Display if an error occurred
        if (!result) {
            RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] FAILED to set pulse width for servo at channel " << static_cast<int>(channel) );
        } else {
            RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] SUCCESS to set pulse width for servo at channel " << static_cast<int>(channel) );
        }
    }



    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr esc_calibration_service_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>(); // Create an object of the custom class we wrote
    rclcpp::spin(node);

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