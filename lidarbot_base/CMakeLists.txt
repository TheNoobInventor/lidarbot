cmake_minimum_required(VERSION 3.8)
project(lidarbot_base)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

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
  # controller_manager
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

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

# EXPORTS 
ament_export_libraries(lidarbot_hardware)

ament_package()