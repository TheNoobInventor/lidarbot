cmake_minimum_required(VERSION 3.8)
project(lidarbot_base)

# find dependencies
find_package(ament_cmake REQUIRED)

# Include C/Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(motor_encoder src/DEV_Config.c src/PCA9685.c src/MotorDriver.c src/motor_encoder.c)

# Install Cpp executables
# Use lines below as a guide
install(TARGETS
  motor_encoder
  DESTINATION lib/${PROJECT_NAME}
)

# Link math, wiringPi and other libraries to motor_encoder executable
target_link_libraries(motor_encoder m wiringPi pthread crypt rt)

# TODO:
# if(BUILD_TESTING)
#   find_package(ament_cmake_mock REQUIRED)
#   find_package(hardware_interface REQUIRED)
# endif()

ament_package()