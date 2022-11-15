/*
 * DCmotor.h
 *
 *  Created on: Nov 14, 2022
 *      Author: Nguyen Hau
 */

#ifndef DCMOTOR_H_
#define DCMOTOR_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stdio.h"
#include "stdbool.h"
#include "math.h"

#define PI 3.141592653589793238462643383279
#define Kp 0.229
#define Ki 15.3
#define Kb 22.222

void ReadEncoder();
void ComputeVelocity();
void PID(float *SetPoint, float* CV, float *current_error,float* PidOutput);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

int current_tick, previous_tick,diff_tick;
int right_count,left_count,right_previous,left_previous,cnt;

float current_error;
float previous_rads_left_velocity,rads_left_velocity,previous_rads_right_velocity,rads_right_velocity;
float rpm_left_velocity,rpm_right_velocity,previous_rpm_left_velocity,previous_rpm_right_velocity;
float previous_pos,pos;
float previous_error,anti_windup_error,reset_error;


/***************TABLE PIN****************/
/*******| LEFT_ENCODER_A  | PE10 |********/
/*******| LEFT_ENCODER_B  | PE11 |********/
/*******| RIGHT_ENCODER_A | PE12 |********/
/*******| RIGHT_ENCODER_B | PE13 |********/



#endif /* INC_DCMOTOR_H_ */
