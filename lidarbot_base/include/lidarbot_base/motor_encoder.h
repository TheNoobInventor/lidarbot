#ifdef __cplusplus
extern "C" {
#endif

#ifndef __MOTOR_ENCODER_H
#define __MOTOR_ENCODER_H__

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <wiringPi.h>

#include "MotorDriver.h"

#define LEFT_WHL_ENCODER 25
#define RIGHT_WHL_ENCODER 24

void handler(int signo);
void left_wheel_pulse();
void right_wheel_pulse();
void set_motor_speeds();
void read_encoder_values(int left_encoder_value, int right_encoder_value);
void calculate_encoder_angle();

#endif 

#ifdef __cplusplus
}
#endif