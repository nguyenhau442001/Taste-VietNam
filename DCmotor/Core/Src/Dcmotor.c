/*
 * Dcmotor.c
 *
 *  Created on: Nov 14, 2022
 *      Author: Nguyen Hau
 *
 *
 */

#include "DCmotor.h"

int current_tick, previous_tick,diff_tick;
int right_count,left_count,right_previous,left_previous,cnt;

double ActualAngularVelocity[2];
double ActualLinearVelocity[2]   ;
double SetPointLinearVelocity[2] ;
double SetPointAngularVelocity[2];

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 static unsigned char state0,state1,state2,state3;
 static bool LEFT_ENCODER_A,RIGHT_ENCODER_A, LEFT_ENCODER_B,RIGHT_ENCODER_B;
 /* MOTOR A */
 if (GPIO_Pin == GPIO_PIN_12)
 {
   // chương trình ngắt của chân 12

	 LEFT_ENCODER_A=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12);
	 state0=state0|LEFT_ENCODER_A;

	 state0=state0<<1;
	 LEFT_ENCODER_B=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13);
	 state0=state0|LEFT_ENCODER_B;
	 state0=state0 & 0x03;

	 switch(state0)
	 {
		 	 	 	 	 case 0:
							 	 if(left_previous==1){left_count++;}
							 	 if(left_previous==2) {left_count--;}
							 	 break;
		 	 	 	 	 case 1:
		 	 	 	 		 	 if(left_previous==3){left_count++;}
		 	 	 	 		 	 if(left_previous==0){left_count--;}
		 	 	 	 		 	 break;
		 	 	 	 	 case 2:
		 	 	 	 		 	 if(left_previous==0){left_count++;}
		 	 	 	 		 	 if(left_previous==3) {left_count--;}
		 	 	 	 		 	 break;
		 	 	 	 	 case 3:
		 	 	 	 		 	 if(left_previous==2){left_count++;}
		 	 	 	 		 	 if(left_previous==1) {left_count--;}
		 	 	 	 		 	 break;
	 }
	 left_previous=state0;
 	 }

	 else if (GPIO_Pin == GPIO_PIN_13)
	 {
	   // chương trình ngắt của chân 13
		 LEFT_ENCODER_A=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12);
		 state1=state1|LEFT_ENCODER_A;

		 state1=state1<<1;
		 LEFT_ENCODER_B=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13);
		 state1=state1|LEFT_ENCODER_B;
		 state1=state1 & 0x03;

		 switch(state1)
		 {
		 	 	 	 	 case 0:
							 	 if(left_previous==1){left_count++;}
							 	 if(left_previous==2) {left_count--;}
							 	 break;
		 	 	 	 	 case 1:
		 	 	 	 		 	 if(left_previous==3){left_count++;}
		 	 	 	 		 	 if(left_previous==0){left_count--;}
		 	 	 	 		 	 break;
		 	 	 	 	 case 2:
		 	 	 	 		 	 if(left_previous==0){left_count++;}
		 	 	 	 		 	 if(left_previous==3) {left_count--;}
		 	 	 	 		 	 break;
		 	 	 	 	 case 3:
		 	 	 	 		 	 if(left_previous==2){left_count++;}
		 	 	 	 		 	 if(left_previous==1) {left_count--;}
		 	 	 	 		 	 break;
		 }
		 left_previous=state1;
	 }

 	 /* MOTOR B */
	 else if (GPIO_Pin == GPIO_PIN_10)		 // LEFT CHANNEL B
	 {
		 // chương trình ngắt của chân 10

		 RIGHT_ENCODER_A=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10);
		 state2=state2|RIGHT_ENCODER_A;

		 state2=state2<<1;
		 RIGHT_ENCODER_B=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11);
		 state2=state2|RIGHT_ENCODER_B;
		 state2=state2 & 0x03;

		 switch(state2)
		 {
	 	 	 case 0:
				 if(right_previous==1){right_count++;}
				 if(right_previous==2){right_count--;}
				 break;
	 	 	 case 1:
	 	 		 	 if(right_previous==3){right_count++;}
	 	 		 	 if(right_previous==0){right_count--;}
	 	 		 	 break;
	 	 	 case 2:
	 	 		 	 if(right_previous==0){right_count++;}
	 	 		 	 if(right_previous==3) {right_count--;}
	 	 		 	 break;
	 	 	 case 3:
	 	 		 	 if(right_previous==2){right_count++;}
	 	 		 	 if(right_previous==1) {right_count--;}
	 	 		 	 break;
		 }
		 right_previous=state2;
 }



	 else if (GPIO_Pin == GPIO_PIN_11)
	 {
		 // chương trình ngắt của chân 11

		 RIGHT_ENCODER_A=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10);
		 state3=state3|RIGHT_ENCODER_A;

		 state3=state3<<1;
		 RIGHT_ENCODER_B=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11);
		 state3=state3|RIGHT_ENCODER_B;
		 state3=state3 & 0x03;

		 switch(state3)
		 {
		 	 	 	 	 case 0:
								 if(right_previous==1){right_count++;}
								 if(right_previous==2){right_count--;}
								 break;
		 	 	 	 	 case 1:
		 	 	 	 		 	 if(right_previous==3){right_count++;}
		 	 	 	 		 	 if(right_previous==0){right_count--;}
		 	 	 	 		 	 break;
		 	 	 	 	 case 2:
		 	 	 	 		 	 if(right_previous==0){right_count++;}
		 	 	 	 		 	 if(right_previous==3) {right_count--;}
		 	 	 	 		 	 break;
		 	 	 	 	 case 3:
		 	 	 	 		 	 if(right_previous==2){right_count++;}
		 	 	 	 		 	 if(right_previous==1) {right_count--;}
		 	 	 	 		 	 break;
		 }
		 right_previous=state3;
	 }
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	cnt++;
	if(cnt==(1000*SAMPLE_TIME)) //1 cnt = 0.001s, default:100 = 0.1s
	{

		ActualAngularVelocity[0]   = left_count * 60  / (ENCODER_RESOLUTION*0.001*cnt);
		ActualAngularVelocity[1]   = right_count* 60  / (ENCODER_RESOLUTION*0.001*cnt);

		left_count=0;
		right_count=0;
		cnt=0;
	}
}
float CurrentError;
void PID(PID_TypeDef *uPID,Error_TypeDef *Error,float Kp, float Ki, float Kb, double SetPoint, double ControlledVariable,float *PidOutput)
{
	// PWM mode has the range from 0 to 400.
	float HighLimit = 400, PWM, PWM_hat, uk, ui;
	static float previous_ui;

	/* ~~~~~~~~~~ Set parameter ~~~~~~~~~~ */
	uPID->Kp = Kp;
	uPID->Ki = Ki;
	uPID->Kb = Kb;

	// Calculate the error
	Error->CurrentError= SetPoint-fabs(ControlledVariable);

	// Proportion
	uk = (uPID->Kp) * (Error->CurrentError);

	// Integration
	ui = previous_ui + (uPID->Ki) * (Error->CurrentError) * SAMPLE_TIME;

	PWM = ui+uk;

	if(PWM < HighLimit)
	{
		PWM_hat = PWM;

		Error-> ResetError  = 0;

		*PidOutput   = PWM;
		uPID->PidOutput=PWM;
	}

	if(PWM > HighLimit)
	{
		PWM_hat = HighLimit;

		Error->ResetError = PWM_hat - PWM;

		Error->AntiWindupError = (uPID->Ki) * (Error->CurrentError) + (Error->ResetError)*(uPID->Kb);

		ui=previous_ui + (Error->AntiWindupError) * SAMPLE_TIME;

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

	// Calculate vel of each wheel
	SetPointLinearVelocity[0]    = (2*linear_velocity-angular_velocity*WHEEL_SEPARATION)/2;  // unit: m/s
	SetPointLinearVelocity[1]    = (2*linear_velocity+angular_velocity*WHEEL_SEPARATION)/2;

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
