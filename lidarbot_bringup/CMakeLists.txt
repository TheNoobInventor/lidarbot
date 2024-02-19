cmake_minimum_required(VERSION 3.8)
project(lidarbot_bringup)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# COMPILE
add_library(mpu6050_hardware_interface SHARED
  src/mpu6050_hardware_interface.cpp
  src/mpu6050_lib.cpp
)

target_include_directories(mpu6050_hardware_interface PRIVATE	include)

ament_target_dependencies(mpu6050_hardware_interface ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Link i2c library to mpu6050_hardware_interface target
target_link_libraries(mpu6050_hardware_interface i2c)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface mpu6050_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/mpu6050_hardware_interface
)

install(
  TARGETS mpu6050_hardware_interface
  DESTINATION share/${PROJECT_NAME}
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# EXPORTS
ament_export_libraries(mpu6050_hardware_interface)

include_directories(include)

# Create Cpp executables
add_executable(mpu6050_offsets src/mpu6050_lib.cpp src/mpu6050_offsets.cpp)
add_executable(mpu6050_covariances src/mpu6050_lib.cpp src/mpu6050_covariances.cpp)

# Install Cpp executables
install(TARGETS
  mpu6050_offsets
  mpu6050_covariances
  DESTINATION lib/${PROJECT_NAME}
)

# Link i2c to mpu6050_offsets and mpu6050_covariances targets
target_link_libraries(mpu6050_offsets i2c)
target_link_libraries(mpu6050_covariances i2c)

ament_package()
