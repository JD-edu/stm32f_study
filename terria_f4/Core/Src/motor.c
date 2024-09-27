/*
 * motor.c
 *
 *  Created on: Jul 24, 2024
 *      Author: User
 */
#include "main.h"
#include "motor.h"

float ignore = 300;
float max = 3000;


// Ignore PWM dead band  忽略PWM信号死区
static int16_t Motor_Ignore_Dead_Zone(int16_t pulse)
{
    if (pulse > 0) return pulse + ignore;
    if (pulse < 0) return pulse - ignore;
    return 0;
}

// The PWM port of the motor is initialized  电机PWM口初始化
void Motor_Init(void)
{
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
//    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

//    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
//    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
//    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

// All motors stopped  所有电机停止
void Motor_Stop(uint8_t brake)
{
    if (brake != 0) brake = 1;
    PWM_M1_A = brake * max;
    PWM_M1_B = brake * max;
    PWM_M2_A = brake * max;
    PWM_M2_B = brake * max;

}

void Motor_Set_Pwm(uint8_t id, int16_t speed)
{
    int16_t pulse = Motor_Ignore_Dead_Zone(speed);
    // Limit input  限制输入
    if (pulse >= max)
        pulse = max;
    if (pulse <= -max)
        pulse = -max;

    switch (id)
    {
    case MOTOR_ID_M1:
    {
        pulse = -pulse;
        if (pulse >= 0)
        {
            PWM_M1_A = pulse;
            PWM_M1_B = 0;
        }
        else
        {
            PWM_M1_A = 0;
            PWM_M1_B = -pulse;
        }
        break;
    }
    case MOTOR_ID_M2:
    {
        pulse = -pulse;
        if (pulse >= 0)
        {
            PWM_M2_A = pulse;
            PWM_M2_B = 0;
        }
        else
        {
            PWM_M2_A = 0;
            PWM_M2_B = -pulse;
        }
        break;
    }



    default:
        break;
    }
}
