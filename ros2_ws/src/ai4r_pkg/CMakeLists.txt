cmake_minimum_required(VERSION 3.8)
project(ai4r_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ai4r_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include Cpp "include" directory
include_directories(
  include
  drivers/src
  #${ament_INCLUDE_DIRS}
)

# Create Cpp executable
add_executable(sensor_pcb_parser src/sensor_pcb_parser.cpp)
ament_target_dependencies(sensor_pcb_parser rclcpp ai4r_interfaces std_msgs)

add_executable(minimal_cpp_executable src/minimal_cpp_node.cpp)
ament_target_dependencies(minimal_cpp_executable rclcpp)

add_executable(oop_cpp_executable src/oop_cpp_node.cpp)
ament_target_dependencies(oop_cpp_executable rclcpp)

add_executable(talker src/oop_pub_cpp_node.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/oop_sub_cpp_node.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(servo_pulse_width_sub src/servo_pulse_width_sub_cpp_node.cpp)
ament_target_dependencies(servo_pulse_width_sub rclcpp std_msgs ai4r_interfaces)

add_executable(multiple_pub_sub src/multiple_pub_sub_cpp_node.cpp)
ament_target_dependencies(multiple_pub_sub rclcpp std_msgs ai4r_interfaces)

add_executable(looping src/looping_cpp_node.cpp)
ament_target_dependencies(looping rclcpp std_msgs ai4r_interfaces)

add_executable(traxxas src/traxxas_node.cpp
                          drivers/src/i2c_driver/i2c_driver.cpp
                          drivers/src/pca9685/pca9685.cpp)
ament_target_dependencies(traxxas rclcpp std_msgs ai4r_interfaces)

add_executable(mux_testing src/mux_testing.cpp
                          drivers/src/i2c_driver/i2c_driver.cpp
                          drivers/src/pca9685/pca9685.cpp)
ament_target_dependencies(mux_testing rclcpp std_msgs ai4r_interfaces)



# Install Cpp executables
install(TARGETS
  minimal_cpp_executable
  oop_cpp_executable
  talker
  listener
  servo_pulse_width_sub
  multiple_pub_sub
  looping
  traxxas
  mux_testing
  sensor_pcb_parser
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/minimal_py_node.py
  scripts/oop_py_node.py
  scripts/oop_pub_py_node.py
  scripts/oop_sub_py_node.py
  scripts/keyboard_teleop.py
  scripts/line_detector_node.py
  scripts/LineDetectorCircle.py
  scripts/LineDetectorEllipse.py
  scripts/test_yaml_params.py
  scripts/Utils.py
  scripts/policy_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
