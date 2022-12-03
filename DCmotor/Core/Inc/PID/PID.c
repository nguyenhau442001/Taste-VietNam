/*
 * PID.c
 *
 *  Created on: Nov 14, 2022
 *      Author: Nguyen Hau
 *
 *
 */

#include <PID/PID.h>
#include "math.h"

Counter_TypeDef Count;
PID_TypeDef uPID;
Status_TypeDef status;

float PidOut[2];
float ActualAngularVelocity[2];
float ActualLinearVelocity[2]   ;
float SetPointLinearVelocity[2] ;
float SetPointAngularVelocity[2];


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 static unsigned char state0,state1,state2,state3;
 static bool Left_Channel_A_Status,Left_Channel_B_Status, Right_Channel_A_Status,Right_Channel_B_Status; //falling or rising edge
 /* MOTOR A */
 if (GPIO_Pin == GPIO_PIN_12)
 {
     /* ~~~~~~ Interrupt program of pin 12 ~~~~~~ */
	 Left_Channel_A_Status=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12);
	 state0=state0|Left_Channel_A_Status;

	 state0=state0<<1;
	 Left_Channel_B_Status=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13);
	 state0=state0|Left_Channel_B_Status;
	 state0=state0 & 0x03;

	 switch(state0)
	 {
		 	 	 	 	 case 0:
							 	 if(status.PreviousLeftStatus==1) {Count.CurrentLeftCount++;}
							 	 if(status.PreviousLeftStatus==2) {Count.CurrentLeftCount--;}
							 	 break;
		 	 	 	 	 case 1:
		 	 	 	 		 	 if(status.PreviousLeftStatus==3) {Count.CurrentLeftCount++;}
		 	 	 	 		 	 if(status.PreviousLeftStatus==0) {Count.CurrentLeftCount--;}
		 	 	 	 		 	 break;
		 	 	 	 	 case 2:
		 	 	 	 		 	 if(status.PreviousLeftStatus==0) {Count.CurrentLeftCount++;}
		 	 	 	 		 	 if(status.PreviousLeftStatus==3) {Count.CurrentLeftCount--;}
		 	 	 	 		 	 break;
		 	 	 	 	 case 3:
		 	 	 	 		 	 if(status.PreviousLeftStatus==2) {Count.CurrentLeftCount++;}
		 	 	 	 		 	 if(status.PreviousLeftStatus==1) {Count.CurrentLeftCount--;}
		 	 	 	 		 	 break;
	 }
	 	 status.PreviousLeftStatus = state0;
 	 }

	 else if (GPIO_Pin == GPIO_PIN_13)
	 {
		 /* ~~~~~~ Interrupt program of pin 13 ~~~~~~ */
		 Left_Channel_A_Status=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12);
		 state1=state1|Left_Channel_A_Status;

		 state1=state1<<1;
		 Left_Channel_B_Status=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13);
		 state1=state1|Left_Channel_B_Status;
		 state1=state1 & 0x03;

		 switch(state1)
		 {
		 	 	 	 	 case 0:
		 	 	 	 		 	 if(status.PreviousLeftStatus==1) {Count.CurrentLeftCount++;}
		 	 	 	 		 	 if(status.PreviousLeftStatus==2) {Count.CurrentLeftCount--;}
						 	 	 break;
	 	 	 	 		 case 1:
	 	 	 	 		 	 	 if(status.PreviousLeftStatus==3) {Count.CurrentLeftCount++;}
	 	 	 	 		 	 	 if(status.PreviousLeftStatus==0) {Count.CurrentLeftCount--;}
	 	 	 	 		 	 	 break;
	 	 	 	 		 case 2:
	 	 	 	 			 	 if(status.PreviousLeftStatus==0) {Count.CurrentLeftCount++;}
	 	 	 	 			 	 if(status.PreviousLeftStatus==3) {Count.CurrentLeftCount--;}
	 	 	 	 			 	 break;
	 	 	 	 		 case 3:
	 	 	 	 		 	 	 if(status.PreviousLeftStatus==2) {Count.CurrentLeftCount++;}
	 	 	 	 		 	 	 if(status.PreviousLeftStatus==1) {Count.CurrentLeftCount--;}
	 	 	 	 		 	 	 break;
		 }
		 status.PreviousLeftStatus = state1;
	 }

 	 /* MOTOR B */
	 else if (GPIO_Pin == GPIO_PIN_10)
	 {
		 /* ~~~~~~ Interrupt program of pin 10 ~~~~~~ */

		 Right_Channel_A_Status=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10);
		 state2=state2|Right_Channel_A_Status;

		 state2=state2<<1;
		 Right_Channel_B_Status=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11);
		 state2=state2|Right_Channel_B_Status;
		 state2=state2 & 0x03;

		 switch(state2)
		 {
		 	 	 	 	 case 0:
		 	 	 	 		 	 if(status.PreviousRightStatus==1) {Count.CurrentRightCount++;}
		 	 	 	 		 	 if(status.PreviousRightStatus==2) {Count.CurrentRightCount--;}
		 	 	 	 		 	 break;
		 	 	 	 	 case 1:
		 	 	 	 		 	 if(status.PreviousRightStatus==3) {Count.CurrentRightCount++;}
		 	 	 	 		 	 if(status.PreviousRightStatus==0) {Count.CurrentRightCount--;}
		 	 	 	 		 	 break;
		 	 	 	 	 case 2:
		 	 	 	 		 	 if(status.PreviousRightStatus==0) {Count.CurrentRightCount++;}
		 	 	 	 		 	 if(status.PreviousRightStatus==3) {Count.CurrentRightCount--;}
		 	 	 	 		 	 break;
		 	 	 	 	 case 3:
		 	 	 	 		 	 if(status.PreviousRightStatus==2) {Count.CurrentRightCount++;}
		 	 	 	 		 	 if(status.PreviousRightStatus==1) {Count.CurrentRightCount--;}
		 	 	 	 		 	 break;
		 }
		 status.PreviousRightStatus = state2;
 }



	 else if (GPIO_Pin == GPIO_PIN_11)
	 {
		 /* ~~~~~~ Interrupt program of pin 11 ~~~~~~ */

		 Right_Channel_A_Status=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10);
		 state3=state3|Right_Channel_A_Status;

		 state3=state3<<1;
		 Right_Channel_B_Status=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11);
		 state3=state3|Right_Channel_B_Status;
		 state3=state3 & 0x03;

		 switch(state3)
		 {
		 	 	 	 case 0:
		 	 	 		 	 if(status.PreviousRightStatus==1) {Count.CurrentRightCount++;}
		 	 	 		 	 if(status.PreviousRightStatus==2) {Count.CurrentRightCount--;}
		 	 	 		 	 break;
		 	 	 	 case 1:
		 	 	 		 	 if(status.PreviousRightStatus==3) {Count.CurrentRightCount++;}
		 	 	 		 	 if(status.PreviousRightStatus==0) {Count.CurrentRightCount--;}
		 	 	 		 	 break;
		 	 	 	 case 2:
		 	 	 		 	 if(status.PreviousRightStatus==0) {Count.CurrentRightCount++;}
		 	 	 		 	 if(status.PreviousRightStatus==3) {Count.CurrentRightCount--;}
		 	 	 		 	 break;
		 	 	 	 case 3:
		 	 	 		 	 if(status.PreviousRightStatus==2) {Count.CurrentRightCount++;}
		 	 	 		 	 if(status.PreviousRightStatus==1) {Count.CurrentRightCount--;}
		 	 	 		 	 break;
		 }
		 status.PreviousRightStatus = state3;
	     }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* ~~~~~ Increase count variable until reach the sample time ~~~~~ */
	Count.SampleTimeCount++;
	if(Count.SampleTimeCount == 1000*(uPID.SampleTime)) //1 step time = 0.001s, default:100 = 0.1s
	{
		/* ~~~~~ Compute angular velocity base the number pulses encoder return ~~~~~ */
		ActualAngularVelocity[0]   = Count.CurrentLeftCount * 60  / (ENCODER_RESOLUTION*0.001*Count.SampleTimeCount);
		ActualAngularVelocity[1]   = Count.CurrentRightCount* 60  / (ENCODER_RESOLUTION*0.001*Count.SampleTimeCount);

		/* ~~~~~ Reset count variables to prepare for the next computation ~~~~~ */
		Count.CurrentLeftCount=0;
		Count.CurrentRightCount=0;
		Count.SampleTimeCount=0;
	}
}

void PID_Compute(PID_TypeDef *uPID,Error_TypeDef *Error,float Kp, float Ki, float Kb, float SampleTime,float RPMSetPoint, float RPMResponse,float *PidOutput)
{
	// PWM mode has the range from 0 to 400.
	float HighLimit = 400, PWM, PWM_hat, uk, ui;
	static float previous_ui;

	/* ~~~~~~~~~~ Set parameter ~~~~~~~~~~ */
	uPID->Kp		= Kp;
	uPID->Ki 		= Ki;
	uPID->Kb 		= Kb;
	uPID->SampleTime= SampleTime;

	/* ~~~~~~~~~~ Calculate the error ~~~~~~~~~~ */
	Error->CurrentError = RPMSetPoint - fabs(RPMResponse);

	/* ~~~~~~~~~~ PWM output for ONLY Proportion ~~~~~~~~~~ */
	uk = (uPID->Kp) * (Error->CurrentError);

	/* ~~~~~~~~~~ PWM output for ONLY Integral~~~~~~~~~~ */
	ui = previous_ui + (uPID->Ki) * (Error->CurrentError) * (uPID->SampleTime);

	/* ~~~~~~~~~~ Sum PWM output from Integral and Proportion ~~~~~~~~~~ */
	PWM = ui+uk;

	/* ~~~~~~~~~~ To avoid accumulate error due to Integral, we apply Anti wind-up method ~~~~~~~~~~ */
	/* ~~~~~ Start Anti-windup ~~~~~ */
	if(PWM < HighLimit)
	{
		PWM_hat = PWM;

		Error-> ResetError  = 0;

		*PidOutput   = PWM;
	}

	if(PWM > HighLimit)
	{
		PWM_hat = HighLimit;

		Error->ResetError = PWM_hat - PWM;

		Error->AntiWindupError = (uPID->Ki) * (Error->CurrentError) + (Error->ResetError)*(uPID->Kb);

		ui=previous_ui + (Error->AntiWindupError) * (uPID->SampleTime);

		*PidOutput = uk+ui;
	}
	previous_ui=ui;
	/* ~~~~~ End Anti-windup ~~~~~ */
}

void PID2Motor(PID_TypeDef *uPID,Error_TypeDef *Error,float Kp, float Ki, float Kb, float SampleTime,float ArraySetpoint[2],float ArrayResponse[2],float PidOutput[2])
{
	/* ~~~~~ Calculate PWM for left motor ~~~~~ */
	PID_Compute(uPID,Error,Kp,Ki,Kb,SampleTime,ArraySetpoint[0],ArrayResponse[0],&PidOutput[0]);

	/* ~~~~~ Calculate PWM for right motor ~~~~~ */
	PID_Compute(uPID,Error,Kp,Ki,Kb,SampleTime,ArraySetpoint[1],ArrayResponse[1],&PidOutput[1]);

	/* ~~~~~ Sampling Time for PID ~~~~~ */
	/* Note: The sampling time for PID and calculate velocity are the same */
	HAL_Delay(1000*(uPID->SampleTime));

	/* ~~~~~ Because the last argument of __HAL_TIM_SetCompare must be INTEGER, so we need round the value PWM output ~~~~~ */
	PidOutput[0]=fabs(round(PidOutput[0]));
	PidOutput[1]=fabs(round(PidOutput[1]));

	/* ~~~~~ Put the PWM value into both motors ~~~~~~ */
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,PidOutput[0]);
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,PidOutput[1]);
}
void ReadEncoder()
{
	/* ~~~~~ Read Encoder base on external interrupt event ~~~~~~ */
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
}

void ComputeVelocity()
{
	/* ~~~~~ Compute Velocity base on timer interrupt event ~~~~~~ */
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
}

void SubcribeVelocityFromRos(const double linear_velocity,const double angular_velocity)
{

	/* ~~~~~~ Calculate linear velocity of each wheel corresponding velocity pair that ROS send down, [0]: left, [1]: right ~~~~~~ */
	SetPointLinearVelocity[0]    =  (2*linear_velocity-angular_velocity*WHEEL_SEPARATION)/2;  // unit: m/s
	SetPointLinearVelocity[1]    =  (2*linear_velocity+angular_velocity*WHEEL_SEPARATION)/2;

	/* ~~~~~~ Convert m/s to rad/s, v=omega.r => omega=v/r (rad/s) ~~~~~ */
	SetPointAngularVelocity[0]   =  SetPointLinearVelocity[0] /WHEEL_RADIUS;
	SetPointAngularVelocity[1]   =  SetPointLinearVelocity[1] /WHEEL_RADIUS;

	/* ~~~~~~  convert to RPM ~~~~~~ */
	SetPointAngularVelocity[0]   = SetPointAngularVelocity[0]*60 /2*PI;
	SetPointAngularVelocity[1]   = SetPointAngularVelocity[1]*60 /2*PI;

	/* ~~~~~~ Determine the direction with the sign of velocity corresponding ~~~~~~ */
	/* ~~~~~~ Note: (0,1): clockwise, (1,0): counter clockwise ~~~~~~ */
	/* ~~~~~~ IN1 (PB1), IN2 (PB2) pin (motor A) ~~~~~~ */
	/* ~~~~~~ IN3 (PE8), IN4 (PE9) pin (motor B) ~~~~~~ */

	if((SetPointLinearVelocity[0]>0) && (SetPointLinearVelocity[1]>0))
	{
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);
	          HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
	}

	if((SetPointLinearVelocity[0]<0) && (SetPointLinearVelocity[1]<0))
	{
		  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);
	          HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);

		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
	}

	if((SetPointLinearVelocity[0]>0) && (SetPointLinearVelocity[1]<0))
	{
		  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
	          HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
	}

	if((SetPointLinearVelocity[0]<0) && (SetPointLinearVelocity[1]>0))
	{
		  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);
	          HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);

		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
	}
}

void PositionControl(PID_TypeDef *uPID,Error_TypeDef *Error,float Kp, float Ki, float Kb, float SampleTime,float PositionSetPoint, float PositionResponse,float *PidPostitionOutput)
{
	//6.227,2.261
	PID_Compute(uPID,Error,Kp,Ki,Kb,SampleTime,PositionSetPoint,PositionResponse,PidPostitionOutput);
}
