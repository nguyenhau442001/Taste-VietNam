/*
------------------------------------------------------------------------------
~ File       		: DCmotor.h
~ Author     		: Nguyen Hau
~ Created on 		: Nov 14, 2022
~ Brief     		: This library is aimed to do some works:
 	 	 	   	   	   1. Read Encoder x2 pulses mode (both falling/rising edge) using external interrupt
 	 	 	   	   	   2. Calculate the actual velocity (RPM) using Timer 2.
 	 	 	   	   	   3. Using PID to compute the value PWM corresponding with desired velocity. (using Timer 3 & PI controller )
~ Implementation 	: in file c
 ------------------------------------------------------------------------------
 */

#ifndef INC_DCMOTOR_H_
#define INC_DCMOTOR_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdbool.h"
#include "math.h"

#define PI 3.141592653589793238462643383279
#define WHEEL_SEPARATION 0.3
#define WHEEL_RADIUS 0.05
#define ENCODER_RESOLUTION 5376

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* PID Structure */
typedef struct
{
	float Kp; 			// default: 0.229
	float Ki; 			// default: 15.3
	float Kb; 			// default: 22.222
	float SampleTime;	// default: 0.05
}PID_TypeDef;

/* Error Structure */
typedef struct
{
	float CurrentError;
	float AntiWindupError;
	float ResetError;
}Error_TypeDef;

/* Tick Structure */
typedef struct
{
	int PreviousTick;
	int CurrentTick;
	int DifferentTick;
}Tick_TypeDef;

/* Counter Structure */
typedef struct
{
	int CurrentLeftCount;
	int CurrentRightCount;
	int SampleTimeCount;
}Counter_TypeDef;

/* Status Read Encoder Structure */
typedef struct
{
	int PreviousLeftStatus;
	int PreviousRightStatus;
}Status_TypeDef;


/* ::::::: Read Encoder x2 Mode ::::::: */
void ReadEncoder();

/* ::::::: ComputeVelocity ::::::: */
void ComputeVelocity();

/* ::::::: Calculate PID for both 2 motor ::::::: */
void PID2Motor(PID_TypeDef *uPID,Error_TypeDef *Error,float Kp, float Ki, float Kb, float SampleTime,float ArraySetpoint[2],float ArrayResponse[2],float PidOutput[2]);

/* ::::::: Calculate PID for single motor ::::::: */
void PID_Compute(PID_TypeDef *uPID,Error_TypeDef *Error,float Kp, float Ki, float Kb, float SampleTime,float RPMSetPoint, float RPMResponse,float *PidOutput);

/* ::::::: Subscribe the pair velocity from ROS ::::::: */
void SubcribeVelocityFromRos(const double linear_velocity,const double angular_velocity);


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#endif /* INC_DCMOTOR_H_ */
