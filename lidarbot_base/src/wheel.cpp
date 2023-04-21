#include "lidarbot_base/wheel.hpp"

Wheel::Wheel(const std::string &wheel_name, int ticks_per_rev)
{
    setup(wheel_name, ticks_per_rev);
}

void Wheel::setup(const std::string &wheel_name, int ticks_per_rev)
{
    name = wheel_name;
    rads_per_tick = (2*M_PI)/ticks_per_rev;
}

double Wheel::calculate_encoder_angle()
{
    return encoder_ticks * rads_per_tick;
}
