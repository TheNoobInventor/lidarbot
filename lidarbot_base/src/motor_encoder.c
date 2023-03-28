#include "lidarbot_base/motor_encoder.h"

// Initialize pulse counters
int left_wheel_pulse_count = 0;
int right_wheel_pulse_count = 0;

//
void read_encoder_values(int left_encoder_value, int right_encoder_value)
{
    left_encoder_value = left_wheel_pulse_count;
    right_encoder_value = right_wheel_pulse_count;
}

//
void set_motor_speeds()
{}

//
void calculate_encoder_angle()
{}

// Left wheel callback function
void left_wheel_pulse()
{   
    // MOTORA - left motor
    // 1 - forward
    // 0 - backward
    if(Motor_Direction(MOTORA) == 1)
        left_wheel_pulse_count++;
    else if(Motor_Direction(MOTORA) == 0)
        left_wheel_pulse_count--;
}

// Right wheel callback function
void right_wheel_pulse()
{
    // MOTORB - right motor
    // 1 - forward
    // 0 - backward
    if(Motor_Direction(MOTORB) == 1)
        right_wheel_pulse_count++;
    else if(Motor_Direction(MOTORB) == 0)
        right_wheel_pulse_count--;
}

void handler(int signo)
{
    DEBUG("\r\nHandler: Motor Stop\r\n");
    Motor_Stop(MOTORA);
    Motor_Stop(MOTORB);

    exit(0);
}