#include <functional>
#include <memory>
#include <stdio.h>
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"
#include "ai4r_interfaces/msg/imu.hpp"
#include "ai4r_interfaces/msg/tof_sensor.hpp"

class SensorPcbParserNode: public rclcpp::Node 
{
    public:
        SensorPcbParserNode() : Node("sensor_pcb_parser_node") {
            // Publish to multiple topics
            imu_topic_pub_ = this->create_publisher<ai4r_interfaces::msg::Imu>("IMU", rclcpp::QoS(10));
            tof1_topic_pub_ = this->create_publisher<ai4r_interfaces::msg::TofSensor>("TOF1", rclcpp::QoS(10));
            tof2_topic_pub_ = this->create_publisher<ai4r_interfaces::msg::TofSensor>("TOF2", rclcpp::QoS(10));
            tof3_topic_pub_ = this->create_publisher<ai4r_interfaces::msg::TofSensor>("TOF3", rclcpp::QoS(10));
            tof4_topic_pub_ = this->create_publisher<ai4r_interfaces::msg::TofSensor>("TOF4", rclcpp::QoS(10));
            tof5_topic_pub_ = this->create_publisher<ai4r_interfaces::msg::TofSensor>("TOF5", rclcpp::QoS(10));
            tof6_topic_pub_ = this->create_publisher<ai4r_interfaces::msg::TofSensor>("TOF6", rclcpp::QoS(10));
            tof7_topic_pub_ = this->create_publisher<ai4r_interfaces::msg::TofSensor>("TOF7", rclcpp::QoS(10));
            tof8_topic_pub_ = this->create_publisher<ai4r_interfaces::msg::TofSensor>("TOF8", rclcpp::QoS(10));


            FILE* serial_port = fopen("/dev/ttyACM0", "r");

            // Allocate memory for read buffer
            char read_buf [256];

            memset(&read_buf, '\0', sizeof(read_buf));

            /*
            Messages are ASCII Encoded from the pico board as follows:

            IMU u_sec_since_last_poll yaw pitch roll accelx accely accelz magx magy magz
            TOF tof_num u_sec_since_last_poll distance_mm_1 ... distance_mm_16

            */

            char *token;
            auto msg = ai4r_interfaces::msg::Imu();
            while(1) {
                while(fgets(read_buf, sizeof(read_buf),serial_port)!= NULL) {
                    if(read_buf[0] != '\n' && read_buf[1] != '\0'){
                        token = strtok(read_buf," ");
                        if (!strcmp(token,"IMU\0")) {
                            token = strtok(NULL," ");
                            if(!strcmp(token,"G\0")) {                            auto msg = ai4r_interfaces::msg::Imu();
                                msg.usec_since_last_gyro_msg = atoll(strtok(NULL," ")); 
                                msg.roll = atof(strtok(NULL," ")); 
                                msg.pitch = atof(strtok(NULL," "));
                                msg.yaw =  atof(strtok(NULL," "));
                                
                            } else if (!strcmp(token,"A\0")) {
                                msg.usec_since_last_accel_msg = atoll(strtok(NULL," ")); 
                                msg.accelx = atof(strtok(NULL," "));
                                msg.accely = atof(strtok(NULL," "));
                                msg.accelz =  atof(strtok(NULL," "));
                            
                            } else if (!strcmp(token,"M\0")) {
                                msg.usec_since_last_mag_msg = atoll(strtok(NULL," ")); 
                                msg.magx = atof(strtok(NULL," "));
                                msg.magy = atof(strtok(NULL," "));
                                msg.magz =  atof(strtok(NULL," "));
                                msg.mag_accuracy = atoi(strtok(NULL," "));
                            }
                            imu_topic_pub_->publish(msg);

                        } else if (!strcmp(token,"TOF\0")) {
                            int tof_num = atoi(strtok(NULL, " "));
                            auto msg = ai4r_interfaces::msg::TofSensor();
                            msg.usec_since_last_msg = atoll(strtok(NULL," "));
                            for (int i = 0; i < 16; i++) {
                                msg.distance_mm[i] = atoi(strtok(NULL," "));
                            }
                        
                            switch (tof_num) {
                                case 0:
                                    tof1_topic_pub_->publish(msg);
                                    break;
                                case 1:
                                    tof2_topic_pub_->publish(msg);
                                    break;
                                case 2:
                                    tof3_topic_pub_->publish(msg);
                                    break;
                                case 3:
                                    tof4_topic_pub_->publish(msg);
                                    break;
                                case 4:
                                    tof5_topic_pub_->publish(msg);
                                    break;
                                case 5:
                                    tof6_topic_pub_->publish(msg);
                                    break;
                                case 6:
                                    tof7_topic_pub_->publish(msg);
                                    break;
                                case 7:
                                    tof8_topic_pub_->publish(msg);
                                    break;

                            }
                        }
                    }
                }
            }
            
            fclose(serial_port);

        }

    private:
        rclcpp::Publisher<ai4r_interfaces::msg::Imu>::SharedPtr imu_topic_pub_;
        rclcpp::Publisher<ai4r_interfaces::msg::TofSensor>::SharedPtr tof1_topic_pub_;
        rclcpp::Publisher<ai4r_interfaces::msg::TofSensor>::SharedPtr tof2_topic_pub_;
        rclcpp::Publisher<ai4r_interfaces::msg::TofSensor>::SharedPtr tof3_topic_pub_;
        rclcpp::Publisher<ai4r_interfaces::msg::TofSensor>::SharedPtr tof4_topic_pub_;
        rclcpp::Publisher<ai4r_interfaces::msg::TofSensor>::SharedPtr tof5_topic_pub_;
        rclcpp::Publisher<ai4r_interfaces::msg::TofSensor>::SharedPtr tof6_topic_pub_;
        rclcpp::Publisher<ai4r_interfaces::msg::TofSensor>::SharedPtr tof7_topic_pub_;
        rclcpp::Publisher<ai4r_interfaces::msg::TofSensor>::SharedPtr tof8_topic_pub_;
        
};

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorPcbParserNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
