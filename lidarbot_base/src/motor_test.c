// Program to test run the motors and output the encoder values.

#include "lidarbot_base/motor_encoder.h"

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
    DEBUG("Left pulse: %d\n", left_wheel_pulse_count);
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
    DEBUG("Right pulse: %d\n", right_wheel_pulse_count);
}

void handler(int signo)
{
    DEBUG("\r\nHandler:Motor Stop\r\n");
    Motor_Stop(MOTORA);
    Motor_Stop(MOTORB);

    exit(0);
}

int main()
{
    // Motor Initialization
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

    // Run motors
    DEBUG("Motor_Run\r\n");
    Motor_Run(MOTORA, FORWARD, 50);
    Motor_Run(MOTORB, BACKWARD, 50);

    // Initialize signal handler for Ctrl+C exception handling
    signal(SIGINT, handler);

    // Keeps program running for the next interrupt
    while(1) {}

    return 0;
}