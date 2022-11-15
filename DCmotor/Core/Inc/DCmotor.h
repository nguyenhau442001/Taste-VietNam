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
#define SAMPLE_TIME 0.1
#define WHEEL_SEPARATION 0.3
#define WHEEL_RADIUS 0.05


void ReadEncoder();
void ComputeVelocity();
void PID(float *SetPoint, float* ControlledVariable,float* PidOutput);
void SubcribeVelocityFromRos(float *linear_velocity,float *angular_velocity,float *left_velocity,float *right_velocity);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);




/***************TABLE PIN****************/
/*******| LEFT_ENCODER_A  | PE10 |********/
/*******| LEFT_ENCODER_B  | PE11 |********/
/*******| RIGHT_ENCODER_A | PE12 |********/
/*******| RIGHT_ENCODER_B | PE13 |********/



#endif /* INC_DCMOTOR_H_ */
