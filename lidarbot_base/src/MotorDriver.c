/*****************************************************************************
* | File        :   MotorDriver.c
* | Author      :   Waveshare team
* | Function    :   Drive TB6612FNG
* | Info        :
*                TB6612FNG is a driver IC for DC motor with output transistor in
*                LD MOS structure with low ON-resistor. Two input signals, IN1
*                and IN2, can choose one of four modes such as CW, CCW, short
*                brake, and stop mode.
*----------------
* |	This version:   V1.0
* | Date        :   2018-09-04
* | Info        :   Basic version
*
******************************************************************************/
#include "lidarbot_base/MotorDriver.h"
#include "lidarbot_base/Debug.h"

UWORD ain1_value, ain2_value; 
UWORD bin1_value, bin2_value;

/**
 * Motor rotation.
 *
 * Example:
 * Motor_Init();
 */
void Motor_Init(void)
{
    PCA9685_Init(0x40);
    PCA9685_SetPWMFreq(50);
}

/**
 * Motor rotation.
 *
 * @param motor: Motor A and Motor B.
 * @param dir: forward and backward.
 * @param speed: Rotation speed.  //(0~100)
 *
 * Example:
 * @code
 * Motor_Run(MOTORA, FORWARD, 50);
 * Motor_Run(MOTORB, BACKWARD, 100);
 */
void Motor_Run(UBYTE motor, DIR dir, UWORD speed)
{
    if(speed > 100)
        speed = 100;

    if(motor == MOTORA) {
        DEBUG("Motor A Speed = %d\r\n", speed);
        PCA9685_SetPwmDutyCycle(PWMA, speed);
        if(dir == FORWARD) {
            DEBUG("forward...\r\n");
            PCA9685_SetLevel(AIN1, 0);
            PCA9685_SetLevel(AIN2, 1);
            ain1_value = 0;
            ain2_value = 1;
        } else {
            DEBUG("backward...\r\n");
            PCA9685_SetLevel(AIN1, 1);
            PCA9685_SetLevel(AIN2, 0);
            ain1_value = 1;
            ain2_value = 0;
        }
    } else {
        DEBUG("Motor B Speed = %d\r\n", speed);
        PCA9685_SetPwmDutyCycle(PWMB, speed);
        if(dir == FORWARD) {
            DEBUG("forward...\r\n");
            PCA9685_SetLevel(BIN1, 0);
            PCA9685_SetLevel(BIN2, 1);
            bin1_value = 0;
            bin2_value = 1;
        } else {
            DEBUG("backward...\r\n");
            PCA9685_SetLevel(BIN1, 1);
            PCA9685_SetLevel(BIN2, 0);
            bin1_value = 1;
            bin2_value = 0;
        }
    }
}

/**
 * Motor stop rotation.
 *
 * @param motor: Motor A and Motor B.
 *
 * Example:
 * @code
 * Motor_Stop(MOTORA);
 */
void Motor_Stop(UBYTE motor)
{
    if(motor == MOTORA) {
        PCA9685_SetPwmDutyCycle(PWMA, 0);
    } else {
        PCA9685_SetPwmDutyCycle(PWMB, 0);
    }
}

/**
* Returns motor direction. 
*   1 - forward 
*   0 - backward
*
* @param motor: Motor A and Motor B
*
* Example:
* @code
* Motor_Direction(MOTORA);
*/
UBYTE Motor_Direction(UBYTE motor)
{
    if(motor == MOTORA) {
        if(ain1_value == 0 && ain2_value == 1)
            return 1;
        else if(ain1_value == 1 && ain2_value == 0)
            return 0;
    }
    else {
        if(bin1_value == 0 && bin2_value == 1)
            return 1;
        else if(bin1_value == 1 && bin2_value == 0)
            return 0;
    }
}
