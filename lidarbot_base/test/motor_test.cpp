// Motor tests to confirm wiring by checking direction of motors in motion

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "lidarbot_base/motor_encoder.h"

#include <cstdlib>
#include <gtest/gtest.h>

// Initialize pulse counters
int left_wheel_pulse_count = 0;
int right_wheel_pulse_count = 0;

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

// Reset pulse counters
void reset_pulse_counters()
{
    right_wheel_pulse_count = 0;
    left_wheel_pulse_count = 0;
}

// Test left motor forward movement
TEST(MotorTest, left_motor_forward)
{
    reset_pulse_counters();
    sleep(2);
    
    // Move left motor forward for 2 seconds at 50% speed
    Motor_Run(MOTORA, FORWARD, 50);
    sleep(2);
    Motor_Stop(MOTORA);

    ASSERT_GT(left_wheel_pulse_count, 0);
}

// Test left motor backward movement
TEST(MotorTest, left_motor_backward)
{
    reset_pulse_counters();
    sleep(2);
    
    // Move left motor backward for 2 seconds at 50% speed
    Motor_Run(MOTORA, BACKWARD, 50);
    sleep(2);
    Motor_Stop(MOTORA);

    ASSERT_LT(left_wheel_pulse_count, 0);
}

// Test right motor forward movement
TEST(MotorTest, right_motor_forward)
{
    reset_pulse_counters();
    sleep(2);
    
    // Move right motor forward for 2 seconds at 50% speed
    Motor_Run(MOTORB, FORWARD, 50);
    sleep(2);
    Motor_Stop(MOTORB);

    ASSERT_GT(right_wheel_pulse_count, 0);
}

// Test right motor backward movement
TEST(MotorTest, right_motor_backward)
{
    reset_pulse_counters();
    sleep(2);
    
    // Move right motor backward for 2 seconds at 50% speed
    Motor_Run(MOTORB, BACKWARD, 50);
    sleep(2);
    Motor_Stop(MOTORB);

    ASSERT_LT(right_wheel_pulse_count, 0);
}

int main(int argc, char **argv)
{
    // Initialize motor driver
    Motor_Init();

    // Initialize wiringPi using GPIO BCM pin numbers
    wiringPiSetupGpio();
    
    // Setup GPIO encoder pins
    pinMode(LEFT_WHL_ENCODER, INPUT);
    pinMode(RIGHT_WHL_ENCODER, INPUT);

    // Setup pull up resistors on encoder pins
    pullUpDnControl(LEFT_WHL_ENCODER, PUD_UP);
    pullUpDnControl(RIGHT_WHL_ENCODER, PUD_UP);

    // Initialize encoder interrupts for falling signal states
    wiringPiISR(LEFT_WHL_ENCODER, INT_EDGE_FALLING, left_wheel_pulse);
    wiringPiISR(RIGHT_WHL_ENCODER, INT_EDGE_FALLING, right_wheel_pulse);

    // Initialize Google Test
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}