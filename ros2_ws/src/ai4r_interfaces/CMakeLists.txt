cmake_minimum_required(VERSION 3.8)
project(ai4r_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # Find the package that generates message code from msg/srv files
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Declare the list of messages you want to generate
set(msg_files
  "msg/ServoPulseWidth.msg"
  "msg/EscAndSteering.msg"
  "msg/ElementInt.msg"
  "msg/IntArray.msg"
  "msg/PixelCoordsFloat32.msg"
  "msg/ImagePointsArray.msg"
  "msg/Imu.msg"
  "msg/TofSensor.msg"
  #DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
  # etc
  )

# Declare the list of services you want to generate
set(srv_files
  # "srv/Service1.srv"
  # "srv/Service2.srv"
   # etc
  )

# Generate the messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

ament_export_dependencies(rosidl_default_runtime)

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

ament_package()
