cmake_minimum_required(VERSION 3.8)
project(lidarbot_bringup)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(imu_sensor_broadcaster_parameters
  src/imu_sensor_broadcaster_parameters.yaml
)

add_library(imu_sensor_broadcaster SHARED
  src/imu_sensor_broadcaster.cpp
  src/MPU6050.cpp
)

target_compile_features(imu_sensor_broadcaster PUBLIC cxx_std_17)
target_include_directories(imu_sensor_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/imu_sensor_broadcaster>
)

target_link_libraries(imu_sensor_broadcaster PUBLIC
  imu_sensor_broadcaster_parameters
)

ament_target_dependencies(imu_sensor_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(
  controller_interface imu_sensor_broadcaster.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_imu_sensor_broadcaster
    test/test_load_imu_sensor_broadcaster.cpp
  )
  target_link_libraries(test_load_imu_sensor_broadcaster
    imu_sensor_broadcaster
	i2c
  )

  ament_target_dependencies(test_load_imu_sensor_broadcaster
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(test_imu_sensor_broadcaster
    test/test_imu_sensor_broadcaster.cpp
  )
  target_link_libraries(test_imu_sensor_broadcaster
    imu_sensor_broadcaster
	i2c
  )

  ament_target_dependencies(test_imu_sensor_broadcaster
    hardware_interface
  )
endif()

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

include_directories(include)

add_executable(mpu6050_test src/MPU6050.cpp src/mpu6050_test.cpp)

install(TARGETS
  mpu6050_test
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(mpu6050_test i2c)

install(
  DIRECTORY include/
  DESTINATION include/imu_sensor_broadcaster
)
install(
  TARGETS
    imu_sensor_broadcaster
    imu_sensor_broadcaster_parameters
  EXPORT export_imu_sensor_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_imu_sensor_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
