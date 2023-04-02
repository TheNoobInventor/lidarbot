#include "lidarbot_base/motor_encoder.h"
#include <math.h>

// Initialize pulse counters
int left_wheel_pulse_count = 0;
int right_wheel_pulse_count = 0;

// Initialize maximum motor rpm
double left_max_rpm = 200.0;
double right_max_rpm = 195.0;

// Wheel diameter
double wheel_diameter = 0.067;

// Read wheel encoder values
void read_encoder_values(int *left_encoder_value, int *right_encoder_value)
{
    *left_encoder_value = left_wheel_pulse_count;
    *right_encoder_value = right_wheel_pulse_count;
}

// Set each motor speed from a respective velocity command interface
void set_motor_speeds(double left_wheel_command, double right_wheel_command)
{   
    // Initialize DIR enum variables 
    DIR left_wheel_direction;
    DIR right_wheel_direction;

    // Convert meters/sec into RPM: for each revolution, a wheel travels
    // pi * diameter meters, and each minute has 60 seconds.
    double left_target_rpm = (left_wheel_command * 60.0) / (M_PI * wheel_diameter);
    double right_target_rpm = (right_wheel_command * 60.0) / (M_PI * wheel_diameter);

    // Scale target motor speeds
    double left_motor_speed = (left_target_rpm / left_max_rpm) * 100.0;
    double right_motor_speed = (right_target_rpm / right_max_rpm) * 100.0;

    // Clip speeds to +/- 45%
    left_motor_speed = fmax(fmin(left_motor_speed, 45.0), -45.0);
    right_motor_speed = fmax(fmin(right_motor_speed, 45.0), -45.0);

    // Set motor directions
    if(left_motor_speed > 0) 
        left_wheel_direction = FORWARD;
    else
        left_wheel_direction = BACKWARD;

    if(right_motor_speed > 0)
        right_wheel_direction = FORWARD;
    else
        right_wheel_direction = BACKWARD;

    // Run motors with specified direction and speeds
    Motor_Run(MOTORA, left_wheel_direction, (int) abs(left_motor_speed));
    Motor_Run(MOTORB, right_wheel_direction, (int) abs(right_motor_speed));
}

// Left wheel callback function
void left_wheel_pulse()
{   
    // MOTORA - left motor
    // 1 - forward
    // 0 - backward
    if(Motor_Direction(MOTORA) == 1)
        left_wheel_pulse_count++;
    else 
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
    else 
        right_wheel_pulse_count--;
}

void handler(int signo)
{
    DEBUG("\r\nHandler: Motor Stop\r\n");
    Motor_Stop(MOTORA);
    Motor_Stop(MOTORB);

    exit(0);
}