/*
 * Dcmotor.c
 *
 *  Created on: Nov 14, 2022
 *      Author: Nguyen Hau
 *
 *
 */

#include "DCmotor.h"


Counter_TypeDef Count;
PID_TypeDef uPID;
Status_TypeDef status;

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
   // chương trình ngắt của chân 12

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
	   // chương trình ngắt của chân 13
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
	 else if (GPIO_Pin == GPIO_PIN_10)		 // LEFT CHANNEL B
	 {
		 // chương trình ngắt của chân 10

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
		 // chương trình ngắt của chân 11

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
	Count.SampleTimeCount++;
	if(Count.SampleTimeCount == 1000*(uPID.SampleTime)) //1 cnt = 0.001s, default:100 = 0.1s
	{

		ActualAngularVelocity[0]   = Count.CurrentLeftCount * 60  / (ENCODER_RESOLUTION*0.001*Count.SampleTimeCount);
		ActualAngularVelocity[1]   = Count.CurrentRightCount* 60  / (ENCODER_RESOLUTION*0.001*Count.SampleTimeCount);

		Count.CurrentLeftCount=0;
		Count.CurrentRightCount=0;
		Count.SampleTimeCount=0;
	}
}

void PID_Compute(PID_TypeDef *uPID,Error_TypeDef *Error,float Kp, float Ki, float Kb, float SampleTime,double SetPoint, double ControlledVariable,float *PidOutput)
{
	// PWM mode has the range from 0 to 400.
	float HighLimit = 400, PWM, PWM_hat, uk, ui;
	static float previous_ui;

	/* ~~~~~~~~~~ Set parameter ~~~~~~~~~~ */
	uPID->Kp		= Kp;
	uPID->Ki 		= Ki;
	uPID->Kb 		= Kb;
	uPID->SampleTime= SampleTime;
	// Calculate the error
	Error->CurrentError= SetPoint-fabs(ControlledVariable);

	// Proportion
	uk = (uPID->Kp) * (Error->CurrentError);

	// Integration
	ui = previous_ui + (uPID->Ki) * (Error->CurrentError) * (uPID->SampleTime);

	PWM = ui+uk;

	// Anti wind-up for Integration
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

}

void ReadEncoder()
{
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
}

void ComputeVelocity()
{
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
}

void SubcribeVelocityFromRos(const double linear_velocity,const double angular_velocity)
{

	// Calculate vel of each wheel [0]: left, [1]: right
	SetPointLinearVelocity[0]    =  (2*linear_velocity-angular_velocity*WHEEL_SEPARATION)/2;  // unit: m/s
	SetPointLinearVelocity[1]    =  (2*linear_velocity+angular_velocity*WHEEL_SEPARATION)/2;

	//v=omega.r => omega=v/r (rad/s)
	SetPointAngularVelocity[0]   =  SetPointLinearVelocity[0] /WHEEL_RADIUS;
	SetPointAngularVelocity[1]   =  SetPointLinearVelocity[1] /WHEEL_RADIUS;

	// convert to RPM
	SetPointAngularVelocity[0]   = SetPointAngularVelocity[0]*60 /2*PI;
	SetPointAngularVelocity[1]   = SetPointAngularVelocity[1]*60 /2*PI;

	// Determine the direction with the sign of value corresponding
	// (0,1): clockwise, (1,0): counter clockwise.
	// IN1 (PB1), IN2 (PB2) pin    (motor A)
	// IN3 (PE8), IN4 (PE9) pin	   (motor B)

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
