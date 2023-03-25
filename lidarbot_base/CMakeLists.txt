cmake_minimum_required(VERSION 3.8)
project(lidarbot_base)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_manager REQUIRED)
#
add_library(lidarbot_hardware SHARED src/lidarbot_hardware.cpp src/MotorDriver.c src/motor_encoder.c src/PCA9685.c src/DEV_Config.c)
# target_link_libraries(lidarbot_hardware m wiringPi pthread crypt rt)

target_include_directories(lidarbot_hardware PRIVATE include)

ament_target_dependencies(
  lidarbot_hardware
  hardware_interface
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
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Include C/Cpp "include" directory
# include_directories(include)

# # Create Cpp executable
# add_executable(motor_encoder src/DEV_Config.c src/PCA9685.c src/MotorDriver.c src/motor_encoder.c)

# Install Cpp executables
# Use lines below as a guide
# install(TARGETS
#   motor_encoder
#   DESTINATION lib/${PROJECT_NAME}
# )

# Link math, wiringPi and other libraries to motor_encoder executable
# target_link_libraries(motor_encoder m wiringPi pthread crypt rt)

# TODO:
# if(BUILD_TESTING)
#   find_package(ament_cmake_mock REQUIRED)
#   find_package(hardware_interface REQUIRED)
# endif()

# ADD COMPILE DIRECTIVES FOR TEST

# EXPORTS (should there be more?)
ament_export_libraries(lidarbot_hardware)

ament_package()