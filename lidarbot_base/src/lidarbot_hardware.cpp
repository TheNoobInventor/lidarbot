#include "lidarbot_base/lidarbot_hardware.hpp"

namespace lidarbot_base
{

LidarbotHardware::LidarbotHardware()
    : logger_(rclcpp::get_logger("LidarbotHardware"))
{}

CallbackReturn LidarbotHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "Initializing...");

    config_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    config_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    config_.enc_ticks_per_rev = std::stoi(info_.hardware_parameters["enc_ticks_per_rev"]);
    config_.loop_rate = std::stod(info_.hardware_parameters["loop_rate"]);

    // Set up wheels
    left_wheel_.setup(config_.left_wheel_name, config_.enc_ticks_per_rev);
    right_wheel_.setup(config_.right_wheel_name, config_.enc_ticks_per_rev);

    RCLCPP_INFO(logger_, "Finished initialization");

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LidarbotHardware::export_state_interfaces()
{
    // Set up a position and velocity interface for each wheel

    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.position));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LidarbotHardware::export_command_interfaces()
{
    // Set up a velocity command for each wheel

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.command));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.command));

    return command_interfaces;
}

CallbackReturn LidarbotHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Configuring motors and encoders...");

    // Initialize motor driver
    Motor_Init();

    // Initialize wiringPi using GPIO BCM pin numbers
    wiringPiSetupGpio();
    
    // Setup GPIO encoder interrupt and direction pins
    pinMode(LEFT_WHL_ENC_INT, INPUT);
    pinMode(RIGHT_WHL_ENC_INT, INPUT);
    pinMode(LEFT_WHL_ENC_DIR, INPUT);
    pinMode(RIGHT_WHL_ENC_DIR, INPUT);

    // Setup pull up resistors on encoder interrupt pins
    pullUpDnControl(LEFT_WHL_ENC_INT, PUD_UP);
    pullUpDnControl(RIGHT_WHL_ENC_INT, PUD_UP);

    // Initialize encoder interrupts for falling signal states
    wiringPiISR(LEFT_WHL_ENC_INT, INT_EDGE_FALLING, left_wheel_pulse);
    wiringPiISR(RIGHT_WHL_ENC_INT, INT_EDGE_FALLING, right_wheel_pulse);

    RCLCPP_INFO(logger_, "Successfully configured motors and encoders!");

    return CallbackReturn::SUCCESS;
}

CallbackReturn LidarbotHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Starting controller ...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn LidarbotHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{   
    RCLCPP_INFO(logger_, "Stopping Controller...");

    return CallbackReturn::SUCCESS;
}

return_type LidarbotHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    // Obtain elapsed time
    double delta_seconds = period.seconds();

    // Obtain encoder values
    read_encoder_values(&left_wheel_.encoder_ticks, &right_wheel_.encoder_ticks);

    // Calculate wheel positions and velocities
    double previous_position = left_wheel_.position;
    left_wheel_.position = left_wheel_.calculate_encoder_angle();
    left_wheel_.velocity = (left_wheel_.position - previous_position) / delta_seconds;

    previous_position = right_wheel_.position;
    right_wheel_.position = right_wheel_.calculate_encoder_angle();
    right_wheel_.velocity = (right_wheel_.position - previous_position) / delta_seconds;

    return return_type::OK;
}

return_type LidarbotHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{   
    double left_motor_counts_per_loop = left_wheel_.command / left_wheel_.rads_per_tick / config_.loop_rate;
    double right_motor_counts_per_loop = right_wheel_.command / right_wheel_.rads_per_tick / config_.loop_rate;

    // Send commands to motor driver
    set_motor_speeds(left_motor_counts_per_loop, right_motor_counts_per_loop);

    return return_type::OK;
}

} // namespace lidarbot_base

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    lidarbot_base::LidarbotHardware, 
    hardware_interface::SystemInterface)
