/*
------------------------------------------------------------------------------
~ File       : DCmotor.h
~ Author     : Nguyen Hau
~ Created on : Nov 14, 2022
~ Brief      : This library is aimed to do some works:
 	 	 	   1. Read Encoder x2 pulses mode (both falling/rising edge) using external interrupt
 	 	 	   2. Calculate the actual velocity (RPM) using Timer 2.
 	 	 	   3. Using PID to compute the value PWM corresponding with desired velocity. (using Timer 3 & PI controller )
 ------------------------------------------------------------------------------
 */

#ifndef DCMOTOR_H_
#define DCMOTOR_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdbool.h"
#include "math.h"

#define PI 3.141592653589793238462643383279
#define Kp 15 //0.229
#define Ki 15.3 //15.3
#define Kb 22.222
#define SAMPLE_TIME 0.05
#define WHEEL_SEPARATION 0.3
#define WHEEL_RADIUS 0.05


void ReadEncoder();
void ComputeVelocity();
void PID(float *SetPoint, float* ControlledVariable,float* PidOutput);
void SubcribeVelocityFromRos(const double linear_velocity,const double angular_velocity,float *left_velocity,float *right_velocity);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);




/***************TABLE PIN****************/
/*******| LEFT_ENCODER_A  | PE10 |********/
/*******| LEFT_ENCODER_B  | PE11 |********/
/*******| RIGHT_ENCODER_A | PE12 |********/
/*******| RIGHT_ENCODER_B | PE13 |********/



#endif /* INC_DCMOTOR_H_ */
