

#include "imu_sensor_broadcaster/imu_sensor_broadcaster.hpp"
#include "MPU6050.h"

#include <memory>
#include <string>
  
// Initialize MPU6050 device
MPU6050 device(0x68);

// Variables to store gyro and accel values
float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;


namespace imu_sensor_broadcaster
{
controller_interface::CallbackReturn IMUSensorBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IMUSensorBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  if (params_.sensor_name.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'sensor_name' parameter has to be specified.");
    return CallbackReturn::ERROR;
  }

  if (params_.frame_id.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'frame_id' parameter has to be provided.");
    return CallbackReturn::ERROR;
  }

  imu_sensor_ = std::make_unique<semantic_components::IMUSensor>(
    semantic_components::IMUSensor(params_.sensor_name));
  try
  {
    // register ft sensor data publisher
    sensor_state_publisher_ =
      get_node()->create_publisher<sensor_msgs::msg::Imu>("~/imu", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  realtime_publisher_->lock();
  realtime_publisher_->msg_.header.frame_id = params_.frame_id;
  // convert double vector to fixed-size array in the message
  for (size_t i = 0; i < 9; ++i)
  {
    realtime_publisher_->msg_.orientation_covariance[i] = params_.static_covariance_orientation[i];
    realtime_publisher_->msg_.angular_velocity_covariance[i] =
      params_.static_covariance_angular_velocity[i];
    realtime_publisher_->msg_.linear_acceleration_covariance[i] =
      params_.static_covariance_linear_acceleration[i];
  }
  realtime_publisher_->unlock();
  
  // Initialize MPU6050 device
  //MPU6050 device(0x68);

  // Variables to store gyro and accel values
  //double accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration IMUSensorBroadcaster::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration IMUSensorBroadcaster::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = imu_sensor_->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::CallbackReturn IMUSensorBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  imu_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IMUSensorBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  imu_sensor_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type IMUSensorBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    realtime_publisher_->msg_.header.stamp = time;

	// Get current accelerometer and gyroscopce values
	device.getAccel(&accel_x, &accel_y, &accel_z);
	device.getGyro(&gyro_x, &gyro_y, &gyro_z);

	// Pass acceleration and gyroscope values to imu message
    realtime_publisher_->msg_.linear_acceleration.x = accel_x;
    realtime_publisher_->msg_.linear_acceleration.y = accel_y;
    realtime_publisher_->msg_.linear_acceleration.z = accel_z;
    realtime_publisher_->msg_.angular_velocity.x = gyro_x;
    realtime_publisher_->msg_.angular_velocity.y = gyro_y;
    realtime_publisher_->msg_.angular_velocity.z = gyro_z;
    realtime_publisher_->msg_.header.stamp = time;
    imu_sensor_->get_values_as_message(realtime_publisher_->msg_);
    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace imu_sensor_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  imu_sensor_broadcaster::IMUSensorBroadcaster, controller_interface::ControllerInterface)

