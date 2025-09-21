#include "ai4r_pkg/traxxas_node.hpp"
#include <algorithm>
#include <string>

#ifdef TRAXXAS_USE_PICO_RP2
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <limits.h>
#include <thread>
#include <unistd.h>
#endif

#ifdef TRAXXAS_USE_PICO_RP2
namespace {
std::string find_serial_device_by_path_substr(const std::string &substr) {
    const char *dirpath = "/dev/serial/by-path";
    DIR *dir = opendir(dirpath);
    if (!dir) {
        return {};
    }
    struct dirent *ent;
    while ((ent = readdir(dir)) != nullptr) {
        if (ent->d_name[0] == '.') {
            continue;
        }
        std::string name(ent->d_name);
        if (name.find(substr) != std::string::npos) {
            std::string linkpath = std::string(dirpath) + "/" + name;
            char buf[PATH_MAX];
            ssize_t n = readlink(linkpath.c_str(), buf, sizeof(buf) - 1);
            if (n > 0) {
                buf[n] = '\0';
                std::string rel(buf);
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
}  // namespace
#endif

class TraxxasNode : public rclcpp::Node {
    public:
        TraxxasNode() : Node("traxxas_node") {
            // Declare parameters with default values (which are used if traxxas_node_params.yaml is not loaded on node startup)
            // this->declare_parameter("steering_neutral_pulse_width", DEFAULT_STEERING_NEUTRAL_PULSE_WIDTH);
            // this->declare_parameter("esc_neutral_pulse_width", DEFAULT_ESC_NEUTRAL_PULSE_WIDTH);
            this->declare_parameter("steering_pulse_width_step", DEFAULT_STEERING_PULSE_WIDTH_STEP);
            this->declare_parameter("esc_pulse_width_step", DEFAULT_ESC_PULSE_WIDTH_STEP);
            this->declare_parameter("neutral_esc_required_to_enable", true);
            this->declare_parameter("action_timeout_can_disable", true);
            this->declare_parameter("cv_status_can_disable", true);

            // Get parameter values and store in class variables
            // steering_neutral_pulse_width_ = this->get_parameter("steering_neutral_pulse_width").as_int();
            // esc_neutral_pulse_width_ = this->get_parameter("esc_neutral_pulse_width").as_int();
            steering_pulse_width_step_ = this->get_parameter("steering_pulse_width_step").as_int();
            esc_pulse_width_step_ = this->get_parameter("esc_pulse_width_step").as_int();
            neutral_esc_required_to_enable_ = this->get_parameter("neutral_esc_required_to_enable").as_bool();
            action_timeout_can_disable_ = this->get_parameter("action_timeout_can_disable").as_bool();
            cv_status_can_disable_ = this->get_parameter("cv_status_can_disable").as_bool();

            // Parameter callback
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&TraxxasNode::parametersCallback, this, std::placeholders::_1));

            // Subscription to the ESC and steering percentage set point action
            esc_and_steering_set_point_percent_action_sub_ = this->create_subscription<ai4r_interfaces::msg::EscAndSteeringPercent>(
                "esc_and_steering_set_point_percent_action", rclcpp::QoS(10), std::bind(&TraxxasNode::escAndSteeringSetPointPercentActionSubscriberCallback, this, std::placeholders::_1)
            );
            // Subscription to the request command
            request_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
                "request", rclcpp::QoS(10), std::bind(&TraxxasNode::requestSubscriberCallback, this, std::placeholders::_1)
            );
            // Subscription to the CV status
            cv_status_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
                "cv_status", rclcpp::QoS(10), std::bind(&TraxxasNode::cvStatusSubscriberCallback, this, std::placeholders::_1)
            );
            // Subscription to the steering trim increment/decrement (negative values are anticlockwise)
            steering_trim_increment_sub_ = this->create_subscription<std_msgs::msg::Int16>(
                "steering_trim_increment", rclcpp::QoS(10), std::bind(&TraxxasNode::steeringTrimIncrementSubscriberCallback, this, std::placeholders::_1)
            );
            // Subscription to the steering trim set (negative values are anticlockwise)
            steering_trim_set_sub_ = this->create_subscription<std_msgs::msg::Int16>(
                "steering_trim_set", rclcpp::QoS(10), std::bind(&TraxxasNode::steeringTrimSetSubscriberCallback, this, std::placeholders::_1)
            );

            // Publisher for the FSM state
            state_publisher_ = this->create_publisher<std_msgs::msg::String>("traxxas_state", 1000);
            // Publishers for the latest ESC and steering pulse width commands sent to the motors
            current_esc_pulse_width_publisher_ = this->create_publisher<std_msgs::msg::UInt16>("traxxas_esc_current_pulse_width", 1000);
            current_steering_pulse_width_publisher_ = this->create_publisher<std_msgs::msg::UInt16>("traxxas_steering_current_pulse_width", 1000);
            // Publisher for the latest steering trim
            current_steering_trim_pulse_width_publisher_ = this->create_publisher<std_msgs::msg::Int32>("traxxas_steering_trim_current_pulse_width", 10);
            // Publish the values from the RC switch
            rc_switch_good_signal_publisher_ = this->create_publisher<std_msgs::msg::Int8>("rc_signal_validity", 10);
            rc_switch_mode_signal_publisher_ = this->create_publisher<std_msgs::msg::Int8>("operating_mode", 10);
            rc_switch_request_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("request", 10);


            // Timer callback for the synchronous FSM
            timer_ = this->create_wall_timer(std::chrono::milliseconds(TRAXXAS_FSM_CYCLE_PERIOD_IN_MILLISECS), std::bind(&TraxxasNode::timerCallback, this));


#ifdef TRAXXAS_USE_PICO_RP2
            serial_by_path_substr_ = this->declare_parameter<std::string>("pico_serial_by_path_substr", kDefaultByPathSubstr);
            open_serial();
            running_.store(true);
            reader_ = std::thread([this]() { this->read_loop(); });
#else
            // ================================================
            // I2C INITIALIZATION CODE
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
            bool verbose_display_for_servo_driver_init = false;
            
            // Call the Servo Driver initialisation function
            bool result_servo_init = m_pca9685_servo_driver.initialise_with_frequency_in_hz(new_frequency_in_hz, verbose_display_for_servo_driver_init);
            
            // Report Servo Board Initialisation Result
            if (!result_servo_init) {
                RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] FAILED - while initialising servo driver with I2C address " << static_cast<int>(m_pca9685_servo_driver.get_i2c_address()) );
            } else {
                RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] SUCCESS - while initialising servo driver with I2C address " << static_cast<int>(m_pca9685_servo_driver.get_i2c_address()) );
            }
            // END OF: I2C INITIALIZATION CODE
#endif
        }

        ~TraxxasNode() override {
#ifdef TRAXXAS_USE_PICO_RP2
            running_.store(false);
            if (reader_.joinable()) {
                reader_.join();
            }
            if (fp_in_) {
                std::fclose(fp_in_);
                fp_in_ = nullptr;
            }
            if (fp_out_) {
                std::fclose(fp_out_);
                fp_out_ = nullptr;
            }
#endif
        }

    private:
        int steering_trim = 0;  // Steering neutral pulse width offset
        // int steering_set_point = steering_neutral_pulse_width_;    // Steering target pulse width
        int steering_set_point = DEFAULT_STEERING_NEUTRAL_PULSE_WIDTH;    // Steering target pulse width
        // int latest_steering_command = steering_neutral_pulse_width_;     // Latest steering pulse width command sent to the servo
        int latest_steering_command = DEFAULT_STEERING_NEUTRAL_PULSE_WIDTH;     // Latest steering pulse width command sent to the servo
        // int esc_set_point = esc_neutral_pulse_width_;    // ESC target pulse width
        int esc_set_point = DEFAULT_ESC_NEUTRAL_PULSE_WIDTH;    // ESC target pulse width
        // int latest_esc_command = esc_neutral_pulse_width_;     // Latest ESC pulse width command sent to the motor
        int latest_esc_command = DEFAULT_ESC_NEUTRAL_PULSE_WIDTH;     // Latest ESC pulse width command sent to the motor
        Traxxas_State traxxas_state = Traxxas_State::Disabled;    // Store the current state of the FSM. Initial state is disabled.
        Request request = Request::Empty; // Store latest request command. Initial request is empty.
        int esc_empty_msg_count = 0;    // Counter to store number of empty message cycles for esc
        int steering_empty_msg_count = 0;   // Counter to store number of empty message cycles for steering
        CV_Status cv_status = CV_Status::Good;    // Store the status of the CV system. Initial status is good.
        Calibrate_Stage calibrate_stage = Calibrate_Stage::Inactive;    // Store the current state of calibration. Initial state is inactive.
        int calibrate_fsm_cycle_count = 0;  // Counter for the number of FSM cycles spent in Traxxas_State::Calibrate;

        // For storing parameter values
        // int steering_neutral_pulse_width_;
        // int esc_neutral_pulse_width_;
        int steering_pulse_width_step_;
        int esc_pulse_width_step_;
        bool neutral_esc_required_to_enable_;
        bool action_timeout_can_disable_;
        bool cv_status_can_disable_;

        
        // String for describing the reason for the most recent transition
        std::string reason_for_previous_state_transition = "FSM initialization";

        // Traxxas node synchronous FSM: Periodically called at regular time intervals to trigger state transitions 
        // Moore machine implementation: outputs only based on the current state of the machine, regardless of the input
        void timerCallback() {
            // State transitions first
            // Traxxas FSM transitions
            switch (traxxas_state) {
                case Traxxas_State::Disabled:
                    if (request == Request::Enable) {
                        traxxas_state = Traxxas_State::Pre_enable;
                        reason_for_previous_state_transition = "Received request- Enable";
                        RCLCPP_INFO_STREAM(this->get_logger(), "Transition to Pre-enable | Received request: Enable" );
                    }
                    else if (request == Request::PrepareForCalibration) {
                        traxxas_state = Traxxas_State::Pre_calibrate;
                        reason_for_previous_state_transition = "Received request- Prepare for calibration";
                        RCLCPP_INFO_STREAM(this->get_logger(), "Transition to Pre-calibrate | Received request: Prepare for calibration" );
                    }
                    break;
                case Traxxas_State::Pre_enable:
                    if (request == Request::Disable) {
                        traxxas_state = Traxxas_State::Disabled;
                        reason_for_previous_state_transition = "Received request- Disable";
                        RCLCPP_INFO_STREAM(this->get_logger(), "Transition to Disabled | Received request: Disable" );
                    }
                    else if (!(neutral_esc_required_to_enable_)) {  // Disabling takes precedence over enabling, hence the "else if" rather than "if"
                        traxxas_state = Traxxas_State::Enabled;
                        reason_for_previous_state_transition = "Neutral ESC not required to enable";
                        RCLCPP_INFO_STREAM(this->get_logger(), "Transition to Enabled | Neutral ESC not required to enable" );
                    }
                    else if (neutral_esc_required_to_enable_) {
                        // if (latest_esc_command == esc_neutral_pulse_width_)  {
                        if (latest_esc_command == DEFAULT_ESC_NEUTRAL_PULSE_WIDTH)  {
                            traxxas_state = Traxxas_State::Enabled;
                            reason_for_previous_state_transition = "ESC is neutral, safe to enable";
                            RCLCPP_INFO_STREAM(this->get_logger(), "Transition to Enabled | ESC is neutral, safe to enable" );
                        } else {
                            reason_for_previous_state_transition = "ESC is NOT neutral, cannot enable";
                            RCLCPP_INFO_STREAM(this->get_logger(), "No transition | ESC is NOT neutral, cannot enable" );
                        }
                    }
                    break;
                case Traxxas_State::Enabled:
                    if (request == Request::Disable) {
                        traxxas_state = Traxxas_State::Disabled;
                        reason_for_previous_state_transition = "Received request- Disable";
                        RCLCPP_INFO_STREAM(this->get_logger(), "Transition to Disabled | Received request: Disable" );
                    }
                    if (action_timeout_can_disable_ && esc_empty_msg_count > MIN_EMPTY_MSG_CYCLES_TO_TIMEOUT) {
                        traxxas_state = Traxxas_State::Disabled;
                        reason_for_previous_state_transition = "ESC channel timed out";
                        RCLCPP_INFO_STREAM(this->get_logger(), "Transition to Disabled | ESC channel timed out: Waited " << static_cast<int>(MIN_EMPTY_MSG_CYCLES_TO_TIMEOUT) << " cycles and no message received" );
                    }
                    if (action_timeout_can_disable_ && steering_empty_msg_count > MIN_EMPTY_MSG_CYCLES_TO_TIMEOUT) {
                        traxxas_state = Traxxas_State::Disabled;
                        reason_for_previous_state_transition = "Steering channel timed out";
                        RCLCPP_INFO_STREAM(this->get_logger(), "Transition to Disabled | Steering channel timed out: Waited " << static_cast<int>(MIN_EMPTY_MSG_CYCLES_TO_TIMEOUT) << " cycles and no message received" );
                    }
                    if (cv_status_can_disable_ && cv_status != CV_Status::Good) {
                        traxxas_state = Traxxas_State::Disabled;
                        reason_for_previous_state_transition = "CV status is NOT good";
                        RCLCPP_INFO_STREAM(this->get_logger(), "Transition to Disabled | CV status is NOT good" );
                    }
                    // if (object_detected) then disable
                    break;
                case Traxxas_State::Pre_calibrate:
                    if (request == Request::Disable) {
                        traxxas_state = Traxxas_State::Disabled;
                        reason_for_previous_state_transition = "Received request- Disable";
                        RCLCPP_INFO_STREAM(this->get_logger(), "Transition to Disabled | Received request: Disable" );
                    }
                    else if (request == Request::PerformCalibration) {
                        traxxas_state = Traxxas_State::Calibrate;
                        reason_for_previous_state_transition = "Received request- Perform calibration";
                        RCLCPP_INFO_STREAM(this->get_logger(), "Transition to Calibrate | Received request: Perform calibration" );
                        RCLCPP_INFO_STREAM(this->get_logger(), "Now applying full forward ESC command" );
                        calibrate_stage = Calibrate_Stage::Full_Forward;    // Enter first stage of Traxxas_State::Calibrate
                        calibrate_fsm_cycle_count = 0;  // Reset the cycle count
                    }
                    break;
                case Traxxas_State::Calibrate:
                    if (request == Request::Disable) {
                        traxxas_state = Traxxas_State::Disabled;
                        reason_for_previous_state_transition = "Received request- Disable";
                        RCLCPP_INFO_STREAM(this->get_logger(), "Transition to Disabled | Received request: Disable" );
                    }
                    if (calibrate_stage == Calibrate_Stage::Complete) {
                        traxxas_state = Traxxas_State::Disabled;
                        reason_for_previous_state_transition = "Calibration complete";
                        RCLCPP_INFO_STREAM(this->get_logger(), "Transition to Disabled | Calibration complete" );
                        calibrate_stage = Calibrate_Stage::Inactive;    // Exit Traxxas_State::Calibrate
                    }
                    break;

            }
            // Calibrate FSM transitions
            switch (calibrate_stage) {
                case Calibrate_Stage::Inactive:
                    break;
                case Calibrate_Stage::Full_Forward:
                    if (calibrate_fsm_cycle_count*(1.0) > (CALIBRATE_FULL_FORWARD_PERIOD_IN_MILLISECS*(1.0))/(TRAXXAS_FSM_CYCLE_PERIOD_IN_MILLISECS*(1.0))) {
                        RCLCPP_INFO_STREAM(this->get_logger(), "Now applying full reverse ESC command" );
                        calibrate_stage = Calibrate_Stage::Full_Reverse;
                        calibrate_fsm_cycle_count = 0;  // Reset the cycle count
                    }
                    break;
                case Calibrate_Stage::Full_Reverse:
                    if (calibrate_fsm_cycle_count*(1.0) > (CALIBRATE_FULL_REVERSE_PERIOD_IN_MILLISECS*(1.0))/(TRAXXAS_FSM_CYCLE_PERIOD_IN_MILLISECS*(1.0))) {
                        RCLCPP_INFO_STREAM(this->get_logger(), "Now applying neutral ESC command" );
                        calibrate_stage = Calibrate_Stage::Complete;
                        calibrate_fsm_cycle_count = 0;  // Reset the cycle count
                    }
                    break;
                case Calibrate_Stage::Complete:
                    break;
            }

            // Then enact resulting state behaviour
            switch (traxxas_state) {
                case Traxxas_State::Disabled:
                    //RCLCPP_INFO_STREAM(this->get_logger(), "DISABLED" );
                    // Set ESC and steering back to neutral position
                    esc_set_point = DEFAULT_ESC_NEUTRAL_PULSE_WIDTH;
                    //steering_set_point = DEFAULT_STEERING_NEUTRAL_PULSE_WIDTH;
                    // Call the function to set the pulse widths
                    setEscAndSteeringPulseWidth();
                    // Action timeout counters should be 0 
                    steering_empty_msg_count = 0;
                    esc_empty_msg_count = 0;
                    // Calibrate FSM cycle count should be 0
                    calibrate_fsm_cycle_count = 0;
                    break;
                case Traxxas_State::Pre_enable:
                    //RCLCPP_INFO_STREAM(this->get_logger(), "PRE-ENABLE" );
                    // Set ESC and steering back to neutral position
                    esc_set_point = DEFAULT_ESC_NEUTRAL_PULSE_WIDTH;
                    //steering_set_point = DEFAULT_STEERING_NEUTRAL_PULSE_WIDTH;
                    // Call the function to set the pulse widths
                    setEscAndSteeringPulseWidth();
                    // Action timeout counters should be 0 
                    steering_empty_msg_count = 0;
                    esc_empty_msg_count = 0;
                    // Calibrate FSM cycle count should be 0
                    calibrate_fsm_cycle_count = 0;
                    break;
                case Traxxas_State::Enabled:
                    //RCLCPP_INFO_STREAM(this->get_logger(), "ENABLED" );
                    // Call the function to set the pulse widths
                    setEscAndSteeringPulseWidth();
                    // Only if wheels are spinning, increment the timeout counters
                    // if (latest_esc_command != esc_neutral_pulse_width_) {
                    if (latest_esc_command != DEFAULT_ESC_NEUTRAL_PULSE_WIDTH) {
                        // Increment counters
                        esc_empty_msg_count++;
                        steering_empty_msg_count++;
                    }
                    // Calibrate FSM cycle count should be 0
                    calibrate_fsm_cycle_count = 0;
                    break;
                case Traxxas_State::Pre_calibrate:
                    //RCLCPP_INFO_STREAM(this->get_logger(), "PRE-CALIBRATE" );
                    // Set ESC and steering back to neutral position
                    esc_set_point = DEFAULT_ESC_NEUTRAL_PULSE_WIDTH;
                    steering_set_point = DEFAULT_STEERING_NEUTRAL_PULSE_WIDTH;
                    // Call the function to set the pulse widths
                    setEscAndSteeringPulseWidth();
                    // Action timeout counters should be 0 
                    steering_empty_msg_count = 0;
                    esc_empty_msg_count = 0;
                    // Calibrate FSM cycle count should be 0
                    calibrate_fsm_cycle_count = 0;
                    break;
                case Traxxas_State::Calibrate:
                    //RCLCPP_INFO_STREAM(this->get_logger(), "CALIBRATE" );
                    // Set steering to neutral position
                    steering_set_point = DEFAULT_STEERING_NEUTRAL_PULSE_WIDTH;
                    // ESC calibration
                    switch (calibrate_stage) {
                        case Calibrate_Stage::Inactive:
                            esc_set_point = DEFAULT_ESC_NEUTRAL_PULSE_WIDTH;
                            break;
                        case Calibrate_Stage::Full_Forward:
                            // Full forward
                            esc_set_point = ESC_MAXIMUM_PULSE_WIDTH;
                            break;
                        case Calibrate_Stage::Full_Reverse:
                            // Full reverse
                            esc_set_point = ESC_MINIMUM_PULSE_WIDTH;
                            break;
                        case Calibrate_Stage::Complete:
                            // Neutral (setting ESC to neutral is necessary to complete calibration)
                            // esc_set_point = esc_neutral_pulse_width_;
                            esc_set_point = DEFAULT_ESC_NEUTRAL_PULSE_WIDTH;
                            break;
                    }
                    // Call the function to set the pulse widths
                    setEscAndSteeringPulseWidth();
                    // Action timeout counters should be 0 
                    steering_empty_msg_count = 0;
                    esc_empty_msg_count = 0;
                    // Increment the calibrate FSM cycle count
                    calibrate_fsm_cycle_count++;
                    break;
            }

            // // ROS Log separator to distinguish between different FSM cycles
            // RCLCPP_INFO_STREAM(this->get_logger(), "----------------------------------------" );

            // Reset the request and CV status to default values (to prevent influencing future FSM cycles)
            request = Request::Empty;
            cv_status = CV_Status::Good;

            // Publish the current Traxxas state
            auto traxxas_state_message = std_msgs::msg::String();
            traxxas_state_message.data = "Error";
            switch (traxxas_state) {
                case Traxxas_State::Disabled:
                    traxxas_state_message.data = "Disabled (Reason: " + reason_for_previous_state_transition + ")";
                    break;
                case Traxxas_State::Pre_enable:
                    traxxas_state_message.data = "Pre-enable (Reason: " + reason_for_previous_state_transition + ")";
                    break;
                case Traxxas_State::Enabled:
                    traxxas_state_message.data = "Enabled (Reason: " + reason_for_previous_state_transition + ")";
                    break;
                case Traxxas_State::Pre_calibrate:
                    traxxas_state_message.data = "Pre-calibrate (Reason: " + reason_for_previous_state_transition + ")";
                    break;
                case Traxxas_State::Calibrate:
                    traxxas_state_message.data = "Calibrate (Reason: " + reason_for_previous_state_transition + ")";
                    break;
            }
            state_publisher_->publish(traxxas_state_message);

            // Publish the current steering trim
            auto steering_trim_message = std_msgs::msg::Int32();
            steering_trim_message.data = steering_trim;
            current_steering_trim_pulse_width_publisher_->publish(steering_trim_message);
        }

#ifndef TRAXXAS_USE_PICO_RP2
        // ================================================
        // I2C PWM SEND FUNCTION
        // Send a pulse width on the specified channel.
        void setPWMSignalViaI2c(uint16_t channel, uint16_t pulse_width_in_us) {
            // Call the function to set the desired pulse width
            bool result = m_pca9685_servo_driver.set_pwm_pulse_in_microseconds(channel, pulse_width_in_us);

            // Display if an error occurred otherwise publish the value set
            if (!result) {
                RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] FAILED to set pulse width for servo at channel " << static_cast<int>(channel) );
            }
            else{
                auto message = std_msgs::msg::UInt16();
                message.data = pulse_width_in_us;
                // Publish the value set
                if (channel == STEERING_SERVO_CHANNEL)
                {
                    current_steering_pulse_width_publisher_->publish(message);
                }
                else if (channel == ESC_SERVO_CHANNEL)
                {
                    current_esc_pulse_width_publisher_->publish(message);
                }
            }
        }
        // END OF: I2C PWM SEND FUNCTION
        // ================================================
#endif

#ifdef TRAXXAS_USE_PICO_RP2
        void send_cmd_via_pico(uint16_t esc_us, uint16_t steering_us) {
            pico_drive_us_ = esc_us;
            pico_steer_us_ = steering_us;
            if (!fp_out_) {
                RCLCPP_WARN_ONCE(this->get_logger(), "[TRAXXAS] RP2 serial not open; skipping PWM command");
                return;
            }
            std::fprintf(fp_out_, "RP2 CMD %u %u\n", static_cast<unsigned>(pico_drive_us_), static_cast<unsigned>(pico_steer_us_));
            std::fflush(fp_out_);

            auto esc_msg = std_msgs::msg::UInt16();
            esc_msg.data = pico_drive_us_;
            current_esc_pulse_width_publisher_->publish(esc_msg);

            auto steering_msg = std_msgs::msg::UInt16();
            steering_msg.data = pico_steer_us_;
            current_steering_pulse_width_publisher_->publish(steering_msg);
        }

        void open_serial() {
            std::string dev;
            if (!serial_by_path_substr_.empty()) {
                dev = find_serial_device_by_path_substr(serial_by_path_substr_);
            }
            if (dev.empty()) {
                dev = "/dev/ttyACM0";
            }

            fp_in_ = std::fopen(dev.c_str(), "r");
            if (!fp_in_) {
                RCLCPP_ERROR(this->get_logger(), "[TRAXXAS] Failed to open RP2 serial device for read: %s", dev.c_str());
                return;
            }
            fp_out_ = std::fopen(dev.c_str(), "w");
            if (!fp_out_) {
                RCLCPP_ERROR(this->get_logger(), "[TRAXXAS] Failed to open RP2 serial device for write: %s", dev.c_str());
                std::fclose(fp_in_);
                fp_in_ = nullptr;
                return;
            }
            std::setvbuf(fp_out_, nullptr, _IONBF, 0);
            RCLCPP_INFO(this->get_logger(), "[TRAXXAS] Opened RP2 serial device: %s", dev.c_str());
        }

        void read_loop() {
            if (!fp_in_) {
                return;
            }
            // Static variable for the previous mode value
            static uint8_t prev_mode_value = 0;
            // Inistialize the buffer
            char buf[256];
            // Enter the endless reading loop
            while (rclcpp::ok() && running_.load()) {
                if (!std::fgets(buf, sizeof(buf), fp_in_)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    continue;
                }
                if (buf[0] == '\0' || buf[0] == '\n') {
                    continue;
                }

                // Debugging print
                //RCLCPP_DEBUG(this->get_logger(), "[TRAXXAS] read_loop received: %s", buf);

                char *token = std::strtok(buf, " \r\n");
                if (!token) {
                    continue;
                }

                if (std::strcmp(token, "RCS") == 0) {
                    // Extract the data
                    uint8_t good_value = std::atoi(std::strtok(nullptr, " \r\n"));
                    uint8_t mode_value = std::atoi(std::strtok(nullptr, " \r\n"));
                    // Publish the values
                    auto good_msg = std_msgs::msg::Int8();
                    auto mode_msg = std_msgs::msg::Int8();
                    good_msg.data = good_value;
                    mode_msg.data = mode_value;
                    rc_switch_good_signal_publisher_->publish(good_msg);
                    rc_switch_mode_signal_publisher_->publish(mode_msg);

                    // Publish a disable or enable request if the mode value changed
                    if (prev_mode_value != mode_value) {
                        if (mode_value == 0) {
                            // Publish the disable message if MANUAL mode is selected
                            auto disable_message = std_msgs::msg::UInt8();
                            disable_message.data = 0;
                            rc_switch_request_publisher_->publish(disable_message);
                        } else if (mode_value == 1) {
                            // // Publish the enable message if AUTONOMOUS mode is selected
                            // auto enable_message = std_msgs::msg::UInt8();
                            // enable_message.data = 1;
                            // rc_switch_request_publisher_->publish(enable_message);
                        }
                    }
                    // Update the previous value of the mode
                    prev_mode_value = mode_value;
                }
                // else if (std::strcmp(token, "SPD\0") == 0 || std::strcmp(token, "SPD") == 0) {
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
                else {
                    RCLCPP_INFO(this->get_logger(), "[TRAXXAS] Receive unrecognised data in USB serial read loop");
                }
            }
        }
#endif

        // Convert percentage to PWM
        uint16_t percentageToPulseWidth(float percent_value, uint16_t minimum_pw, uint16_t maximum_pw) {
            uint16_t pulse_width = 0;

            // Linear conversion capped between the range -100 to 100
            if(percent_value <= -100.0) {
                pulse_width = minimum_pw;
            }
            else if(percent_value >= 100.0) {
                pulse_width = maximum_pw;
            }
            else {
                // Basic equation to convert between two ranges. Idea is add fraction of total range to the minimum value.
                float float_in_range = static_cast<float>(minimum_pw) + static_cast<float>(maximum_pw - minimum_pw)*((percent_value + 100.0)/200.0);
                
                // Convert from float to integer
                pulse_width = static_cast<uint16_t>(float_in_range);
            }

            return pulse_width;
        }

        void setEscAndSteeringPulseWidth() {
            // STEERING
            // - Send steering PWM one step closer to the set point
            // - See if the set point is at a distance greater than the defined step away from the latest command.
            //   - If it is, increment the command by the step amount towards the direction of the set point.
            //   - Otherwise, the command is exactly the set point.
            if (abs(steering_set_point - latest_steering_command) > steering_pulse_width_step_) {
                if (steering_set_point > latest_steering_command) {
                    latest_steering_command += steering_pulse_width_step_;
                } else {
                    latest_steering_command -= steering_pulse_width_step_;
                }
            } else {
                latest_steering_command = steering_set_point;
            }

            // Add the steering trim if sending to the steering servo and clamp within servo limits
            int steering_command_with_trim = latest_steering_command + steering_trim;
            int steering_command_to_send = std::clamp(
                steering_command_with_trim,
                static_cast<int>(STEERING_MINIMUM_PULSE_WIDTH),
                static_cast<int>(STEERING_MAXIMUM_PULSE_WIDTH));

            // ESC
            // - Send ESC PWM one step closer to the set point
            // - See if the set point is at a distance greater than the defined step away from the latest command.
            //   - If it is, increment the command by the step amount towards the direction of the set point.
            //   - Otherwise, the command is exactly the set point.
            if (abs(esc_set_point - latest_esc_command) > esc_pulse_width_step_) {
                if (esc_set_point > latest_esc_command) {
                    latest_esc_command += esc_pulse_width_step_;
                } else {
                    latest_esc_command -= esc_pulse_width_step_;
                }
            } else {
                latest_esc_command = esc_set_point;
            }

            int esc_command_to_send = std::clamp(
                latest_esc_command,
                static_cast<int>(ESC_MINIMUM_PULSE_WIDTH),
                static_cast<int>(ESC_MAXIMUM_PULSE_WIDTH));

#ifdef TRAXXAS_USE_PICO_RP2
            send_cmd_via_pico(static_cast<uint16_t>(esc_command_to_send),
                              static_cast<uint16_t>(steering_command_to_send));
#else
            setPWMSignalViaI2c(ESC_SERVO_CHANNEL, static_cast<uint16_t>(esc_command_to_send));
            setPWMSignalViaI2c(STEERING_SERVO_CHANNEL, static_cast<uint16_t>(steering_command_to_send));
#endif
        }

        // For receiving percentage values to control both the steering (channel 0) and esc (channel 1) using the synchronous FSM
        void escAndSteeringSetPointPercentActionSubscriberCallback(const ai4r_interfaces::msg::EscAndSteeringPercent & msg) {
            // Convert to pulse width and save value as the set point
            steering_set_point = percentageToPulseWidth(msg.steering_percent, STEERING_MINIMUM_PULSE_WIDTH, STEERING_MAXIMUM_PULSE_WIDTH);
            esc_set_point = percentageToPulseWidth(msg.esc_percent, ESC_MINIMUM_PULSE_WIDTH, ESC_MAXIMUM_PULSE_WIDTH);

            // Display the message received
            //RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] Message received for steering servo. Percentage command received = " << static_cast<float>(msg.steering_percent) << ", PWM sent to motors = " << static_cast<float>(steering_set_point) );
            //RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] Message received for ESC. Percentage command received = " << static_cast<float>(msg.esc_percent) << ", PWM sent to motors = " << static_cast<float>(esc_set_point) );

            steering_empty_msg_count = 0;   // Message received so reset counter to 0
            esc_empty_msg_count = 0;    // Message received so reset counter to 0
        }

        // For receiving request commands
        void requestSubscriberCallback(const std_msgs::msg::UInt8 & msg) {
            // Extract request
            int incoming = msg.data;

            // Display the message received
            if (incoming == 0) {
                RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] REQUEST DISABLE" );
                request = Request::Disable;
            } else if (incoming == 1) {
                RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] REQUEST ENABLE" );
                request = Request::Enable;
            } else if (incoming == 2) {
                RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] REQUEST PREPARE FOR CALIBRATION" );
                request = Request::PrepareForCalibration;
            } else if (incoming == 3) {
                RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] REQUEST PERFORM CALIBRATION" );
                request = Request::PerformCalibration;
            } else {
                RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] INVALID REQUEST" );
            }
        }

        // For receiving cv-error flag commands
        void cvStatusSubscriberCallback(const std_msgs::msg::UInt8 & msg) {
            // Extract CV status
            int incoming = msg.data;

            // Log if the CV status is NOT good
            if (incoming != 0) {
                RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] CV status is NOT good." );
                cv_status = CV_Status::Error;
            }
        }

        // For receiving steering trim increment/decrement commands
        void steeringTrimIncrementSubscriberCallback(const std_msgs::msg::Int16 & msg) {
            // Extract the increment/decrement value
            int incoming = msg.data;

            // Increment/decrement the steering trim
            steering_trim += incoming;

            // Display the message received and new steering trim value
            RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] Steering trim increment received: " << incoming << ". New steering trim = " << steering_trim );
        }

        // For receiving steering trim set commands
        void steeringTrimSetSubscriberCallback(const std_msgs::msg::Int16 & msg) {
            // Extract the set value
            int incoming = msg.data;

            // Set the steering trim
            steering_trim = incoming;

            // Display the message received and new steering trim value
            RCLCPP_INFO_STREAM(this->get_logger(), "[TRAXXAS] Steering trim set received: " << incoming << ". New steering trim = " << steering_trim );
        }

        // Callback for setting parameters
        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = false;
            result.reason = "";
            for (const auto &param : parameters) {
                // if (param.get_name() == "steering_neutral_pulse_width") {
                //     if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                //         if (param.as_int() > STEERING_MINIMUM_PULSE_WIDTH && param.as_int() < STEERING_MAXIMUM_PULSE_WIDTH) {
                //             steering_neutral_pulse_width_ = param.as_int();
                //             result.successful = true;
                //         }
                //     }
                // }
                // if (param.get_name() == "esc_neutral_pulse_width") {
                //     if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                //         if (param.as_int() > ESC_MINIMUM_PULSE_WIDTH && param.as_int() < ESC_MAXIMUM_PULSE_WIDTH) {
                //             esc_neutral_pulse_width_ = param.as_int();
                //             result.successful = true;
                //         }
                //     }
                // }
#ifdef TRAXXAS_USE_PICO_RP2
                if (param.get_name() == "pico_serial_by_path_substr") {
                    if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                        serial_by_path_substr_ = param.as_string();
                        result.successful = true;
                    }
                }
#endif

                if (param.get_name() == "steering_pulse_width_step") {
                    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                        steering_pulse_width_step_ = param.as_int();
                        result.successful = true;
                    }
                }
                if (param.get_name() == "esc_pulse_width_step") {
                    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                        esc_pulse_width_step_ = param.as_int();
                        result.successful = true;
                    }
                }
                if (param.get_name() == "neutral_esc_required_to_enable") {
                    if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                        neutral_esc_required_to_enable_ = param.as_bool();
                        result.successful = true;
                    }
                }
                if (param.get_name() == "action_timeout_can_disable") {
                    if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                        action_timeout_can_disable_ = param.as_bool();
                        result.successful = true;
                    }
                }
                if (param.get_name() == "cv_status_can_disable") {
                    if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                        cv_status_can_disable_ = param.as_bool();
                        result.successful = true;
                    }
                }
            }
            return result;
        }


        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Subscription<ai4r_interfaces::msg::EscAndSteeringPercent>::SharedPtr esc_and_steering_set_point_percent_action_sub_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr request_sub_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cv_status_sub_;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr steering_trim_increment_sub_;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr steering_trim_set_sub_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
        rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr current_esc_pulse_width_publisher_;
        rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr current_steering_pulse_width_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_steering_trim_pulse_width_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr rc_switch_good_signal_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr rc_switch_mode_signal_publisher_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr rc_switch_request_publisher_;

        OnSetParametersCallbackHandle::SharedPtr callback_handle_;

#ifdef TRAXXAS_USE_PICO_RP2
        std::string serial_by_path_substr_;
        std::FILE *fp_in_ = nullptr;
        std::FILE *fp_out_ = nullptr;
        std::thread reader_{};
        std::atomic<bool> running_{false};
        uint16_t pico_drive_us_ = DEFAULT_ESC_NEUTRAL_PULSE_WIDTH;
        uint16_t pico_steer_us_ = DEFAULT_STEERING_NEUTRAL_PULSE_WIDTH;
        static constexpr const char kDefaultByPathSubstr[] = "usb-0:1.4";
#endif

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TraxxasNode>();
    rclcpp::spin(node); // Use rclcpp::spin() to handle callbacks.

#ifndef TRAXXAS_USE_PICO_RP2
	// Close the I2C device
	bool close_success = m_i2c_driver.close_i2c_device();
	
	// Display the status
	if (!close_success) {
		RCLCPP_INFO_STREAM(node->get_logger(), "[TRAXXAS] FAILED to close I2C device named " << m_i2c_driver.get_device_name());
	} else {
		RCLCPP_INFO_STREAM(node->get_logger(), "[TRAXXAS] Successfully closed device named " << m_i2c_driver.get_device_name());
	}
#endif
    
    rclcpp::shutdown();
    return 0;
}
