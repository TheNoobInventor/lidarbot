cmake_minimum_required(VERSION 3.8)
project(lidarbot_base)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

# COMPILE
add_library(lidarbot_hardware SHARED 
          src/lidarbot_hardware.cpp 
          src/wheel.cpp 
          src/MotorDriver.c 
          src/motor_encoder.c 
          src/PCA9685.c 
          src/DEV_Config.c
)

target_include_directories(lidarbot_hardware PRIVATE include)

ament_target_dependencies(
  lidarbot_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Link WiringPi library to lidarbot_hardware target
target_link_libraries(lidarbot_hardware wiringPi)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface lidarbot_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/lidarbot_hardware
)

install(
  TARGETS lidarbot_hardware
  DESTINATION share/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

include_directories(include)

# Add motor_checks_client and motor_checks_server nodes as executables with dependencies
add_executable(motor_checks_server 
              src/motor_checks_server.cpp 
              src/motor_encoder.c   
              src/DEV_Config.c 
              src/PCA9685.c 
              src/MotorDriver.c) 

ament_target_dependencies(motor_checks_server rclcpp std_srvs)
target_link_libraries(motor_checks_server m wiringPi pthread crypt rt)
target_compile_options(motor_checks_server PRIVATE -w)

add_executable(motor_checks_client src/motor_checks_client.cpp) 
ament_target_dependencies(motor_checks_client rclcpp std_srvs)

# Install executables to enable discovery for 'ros2 run'
install(TARGETS 
    motor_checks_client 
    motor_checks_server
    DESTINATION lib/${PROJECT_NAME}
)

# EXPORTS 
ament_export_libraries(lidarbot_hardware)

ament_package()
