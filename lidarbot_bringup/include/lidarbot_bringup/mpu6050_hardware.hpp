#ifndef _LIDARBOT_BRINGUP__MPU6050_HARDWARE_HPP_
#define _LIDARBOT_BRINGUP__MPU6050_HARDWARE_HPP_

#include <memory>
#include <thread>
#include <vector>
#include <string>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mpu6050_lib.h"

using hardware_interface::return_type;
using hardware_interface::CallbackReturn;

namespace lidarbot_bringup
{

class MPU6050Hardware : public hardware_interface::SensorInterface
{
  public:
    MPU6050Hardware();

    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  private:
	Quaternion orientation;
	double angular_vel_x;
	double angular_vel_y;
	double angular_vel_z;
	double linear_accel_x;
	double linear_accel_y;
	double linear_accel_z;
	  	
	Quaternion quat;
	float euler_angles[3]; 
	float gyro_values[3];
  	float accel_values[3];

    rclcpp::Logger logger_;
};

} // namespace lidarbot_bringup

#endif
