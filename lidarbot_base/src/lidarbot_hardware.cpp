#include "lidarbot_base/lidarbot_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace lidarbot_base
{

LidarbotHardware::LidarbotHardware()
    : logger(rclcpp::get_logger("LidarbotHardware"))
{}

CallbackReturn LidarbotHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger, "Initializing...");

    time = std::chrono::system_clock::now();

    config.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    config.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    config.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    config.enc_ticks_per_rev = std::stoi(info_.hardware_parameters["enc_ticks_per_rev"]);

    // Set up weheels
    left_wheel.setup(config.left_wheel_name, config.enc_ticks_per_rev);
    right_wheel.setup(config.right_wheel_name, config.enc_ticks_per_rev);

    RCLCPP_INFO(logger, "Finished initialization");

    return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> LidarbotHardware::export_state_interfaces()
{
    // Set up a position and a velocity interface for each wheel

    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel.name, hardware_interface::HW_IF_VELOCITY, &left_wheel.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel.name, hardware_interface::HW_IF_POSITION, &left_wheel.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel.name, hardware_interface::HW_IF_VELOCITY, &right_wheel.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel.name, hardware_interface::HW_IF_POSITION, &right_wheel.position));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LidarbotHardware::export_command_interfaces()
{
    // Set up a velocity command for each wheel

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(left_wheel.name, hardware_interface::HW_IF_VELOCITY, &left_wheel.command));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(right_wheel.name, hardware_interface::HW_IF_VELOCITY, &right_wheel.command));

    return command_interfaces;
}

//FIXME:Fill up this method or just collapse into on_init() function?
// CallbackReturn LidarbotHardware::on_configure(const rclcpp_lifecycle::State & previous_state)
CallbackReturn LidarbotHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger, "Configuring PCA9685 PWM motor driver..."); // TODO: Change

    // MOTOR ENCODER SETUP

    RCLCPP_INFO(logger, "Successfully configured motors!"); //TODO:

    return CallbackReturn::SUCCESS;
}

// CallbackReturn LidarbotHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
CallbackReturn LidarbotHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}

// CallbackReturn LidarbotHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
CallbackReturn LidarbotHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}

// return_type LidarbotHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
return_type LidarbotHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    return return_type::OK;
}

// return_type LidarbotHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
return_type LidarbotHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    return return_type::OK;
}

}// namespace lidarbot_base

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    lidarbot_base::LidarbotHardware, hardware_interface::SystemInterface)