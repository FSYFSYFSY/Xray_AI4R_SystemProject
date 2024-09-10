#ifndef PWM_DRIVER_NODE_HPP
#define PWM_DRIVER_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/int16.hpp"

// ai4r custom message types
#include "ai4r_interfaces/msg/esc_and_steering_percent.hpp"
 
// Drivers
#include "i2c_driver/i2c_driver.h"
#include "pca9685/pca9685.h"

#include <math.h>
#include <bitset>

using namespace std::chrono_literals;

// Constants: 

constexpr uint8_t  STEERING_SERVO_CHANNEL = 12;
constexpr uint8_t  ESC_SERVO_CHANNEL      = 15;

// Request types
enum class Request : uint8_t {
    Disable = 0,
    Enable = 1,
    PrepareForCalibration = 2,
    PerformCalibration = 3,
    // Add more request types as needed
    Empty = 255
};

// CV Status types
enum class CV_Status : uint8_t {
    Good = 0,
    Error = 1,
    // Add more CV status types as needed
};

// States of Traxxas Finite State Machine
enum class Traxxas_State {
    Disabled,
    Pre_enable,
    Enabled,
    Pre_calibrate,
    Calibrate,
    // Add more states as needed
};

// Stage of calibration
enum class Calibrate_Stage {
    Inactive,
    Full_Forward,
    Full_Reverse,
    Complete
};


constexpr float    SERVO_FREQUENCY = 100.0f;    // PCA9685 (servo board) outputs 200 Hz by default

constexpr uint16_t TRAXXAS_FSM_CYCLE_PERIOD_IN_MILLISECS = 10;    // Period of one Traxxas FSM cycle. 10ms period is 100Hz (same rate as servo board)
constexpr uint16_t CALIBRATE_FULL_FORWARD_PERIOD_IN_MILLISECS = 2000;    // Period of full forward command during calibration
constexpr uint16_t CALIBRATE_FULL_REVERSE_PERIOD_IN_MILLISECS = 2000;    // Period of full reverse command during calibration

// Pulse widths are in us (micro seconds).

// These MUX_SELECT constants are not used in code, serving only as convenient documentation for input signal specifications for the mux.
// Note there are no requirements or restrictions on the master (M1-M4) and slave (S1-S4) channel inputs, only the select (SELECT) channel input as shown in the constants below.
// PWM signal on the select channel must be between 10 - 330 Hz pulse rate (PWM freq).
constexpr uint16_t MUX_SELECT_MAXIMUM_PULSE_WIDTH = 2500;   // Input PWM signal cannot be more than this
constexpr uint16_t MUX_SELECT_MINIMUM_PULSE_WIDTH = 500;    // Input PWM signal cannot be less than this
constexpr uint16_t MUX_SELECT_DEFAULT_THRESHOLD_PULSE_WIDTH = 1696; // Default threshold parameter 
constexpr uint16_t MUX_SELECT_MAXIMUM_THRESHOLD_PULSE_WIDTH = 2100; // Threshold parameter cannot be more than this
constexpr uint16_t MUX_SELECT_MINIMUM_THRESHOLD_PULSE_WIDTH = 900;  // Threshold parameter cannot be less than this
constexpr uint16_t MUX_SELECT_THRESHOLD_PULSE_WIDTH_MULTIPLE = 16;  // Threshold parameter must be a multiple of 16 us.
// When the Inversion parameter is disabled (by default), pulse width above the threshold selects Slave as input (i.e. 'activates' the switch)

// These define the min, max and default neutral points for each of the servo channels. 
// Any values received outside of the minimum or maximum value defined below will be clamped.
// Percent values are mapped linearly from the range [-100.0,100.0] to pulse width values in the range [MINIMUM_PULSE_WIDTH, MAXIMUM_PULSE_WIDTH]. 
// When the node starts up, the neutral values (defined as Parameters in traxxas_node_params.yaml) are sent to each servo channel.
// The default neutral values are used if traxxas_node_params.yaml is not loaded on node startup.
constexpr uint16_t ESC_MAXIMUM_PULSE_WIDTH =  1975;
constexpr uint16_t ESC_MINIMUM_PULSE_WIDTH =  1015;
constexpr uint16_t DEFAULT_ESC_NEUTRAL_PULSE_WIDTH =  1490;
constexpr uint16_t STEERING_MAXIMUM_PULSE_WIDTH =  1750;
constexpr uint16_t STEERING_MINIMUM_PULSE_WIDTH =  1250;
constexpr uint16_t DEFAULT_STEERING_NEUTRAL_PULSE_WIDTH =  1500;

// Default maximum pulse width step size change per Traxxas FSM cycle. Defined as Parameters in traxxas_node_params.yaml.
// The default values are used if traxxas_node_params.yaml is not loaded on node startup.
constexpr uint16_t DEFAULT_ESC_PULSE_WIDTH_STEP = 250;
constexpr uint16_t DEFAULT_STEERING_PULSE_WIDTH_STEP = 50;

constexpr uint16_t MIN_EMPTY_MSG_CYCLES_TO_TIMEOUT = 50;    // Minimum number of Traxxas FSM cycles (where no message is received) needed to get timeout (automatically switch from Enabled to Disabled state)

// VARIABLES FOR THE PWM DRIVER NODE:

// > I2C Driver Object:
// You will need to change the i2c number
// - List all i2c devices: `ls /dev/i2c*`
// - Allow permissions for the relevant device number e.g. for `/dev/i2c-1` which is i2c pin 1 on the RPi by doing: `sudo chmod 777 /dev/i2c-1` (for read, write, execute access for user, group and others)
const char * m_i2c_device_name = "/dev/i2c-1";
I2C_Driver m_i2c_driver (m_i2c_device_name);

// > PCA9685 PWM Servo Driver Object:
const uint8_t m_pca9685_address = PCA9685_I2C_ADDRESS_DEFAULT;  // Default I2C address is 0x40 (hex) = 64 (decimal)
PCA9685 m_pca9685_servo_driver (&m_i2c_driver, m_pca9685_address);

#endif