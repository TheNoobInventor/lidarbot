cmake_minimum_required(VERSION 3.8)
project(lidarbot_slam)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
