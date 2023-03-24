#ifndef _LIDARBOT_BASE__LIDARBOT_HARDWARE_HPP_
#define _LIDARBOT_BASE__LIDARBOT_HARDWARE_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <cstring>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "motor_encoder.h"
#include "config.hpp"
#include "wheel.hpp"

using hardware_interface::return_type;
using hardware_interface::CallbackReturn;

namespace lidarbot_base
{
    
class LidarbotHardware : public hardware_interface::SystemInterface
{
  public:
    LidarbotHardware(); // confirm this

    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  private:
    Config config;

    Wheel left_wheel;
    Wheel right_wheel;

    rclcpp::Logger logger;

    std::chrono::time_point<std::chrono::system_clock> time;
};

}

#endif