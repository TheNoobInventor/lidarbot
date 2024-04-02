cmake_minimum_required(VERSION 3.8)
project(lidarbot_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/trajectory_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
