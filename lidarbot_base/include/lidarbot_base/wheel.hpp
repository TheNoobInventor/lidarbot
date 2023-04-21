#ifndef _LIDARBOT_BASE__WHEEL_H_
#define _LIDARBOT_BASE__WHEEL_H_

#include <string>
#include <cmath>

class Wheel
{   
  public:
    std::string name = "";
    int encoder_ticks = 0;
    double command = 0.0;
    double position = 0.0;
    double velocity = 0.0;
    double rads_per_tick = 0.0;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int ticks_per_rev);

    void setup(const std::string &wheel_name, int ticks_per_rev);

    double calculate_encoder_angle();

};

#endif
