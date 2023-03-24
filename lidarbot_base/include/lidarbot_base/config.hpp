#ifndef _LIDARBOT_BASE__CONFIG_H_
#define _LIDARBOT_BASE__CONFIG_H_

#include <string>

struct Config
{
    std::string left_wheel_name = "left_wheel";
    std::string right_wheel_name = "right_wheel";
    float loop_rate = 30;
    int enc_ticks_per_rev = 20;
};

#endif