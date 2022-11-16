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
#define SAMPLE_TIME 0.05
#define WHEEL_SEPARATION 0.3
#define WHEEL_RADIUS 0.05
#define ENCODER_RESOLUTION 5376

/* PID Structure */
typedef struct
{
	float Kp; // default: 0.229
	float Ki; // default: 15.3
	float Kb; // default: 22.222
	float PidOutput;
}PID_TypeDef;

/* Error Structure */
typedef struct
{
	float CurrentError;
	float AntiWindupError;
	float ResetError;
}Error_TypeDef;


void ReadEncoder();
void ComputeVelocity();
void PID_Compute(PID_TypeDef *uPID,Error_TypeDef *Error,float Kp, float Ki, float Kb, double SetPoint, double ControlledVariable,float *PidOutput);
void SubcribeVelocityFromRos(const double linear_velocity,const double angular_velocity);









#endif /* INC_DCMOTOR_H_ */
