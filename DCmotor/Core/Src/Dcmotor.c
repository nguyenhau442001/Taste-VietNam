/*
 * Dcmotor.c
 *
 *  Created on: Nov 14, 2022
 *      Author: Nguyen Hau
 *
 *
 */

#include "DCmotor.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 static unsigned char state0,state1,state2,state3;
 static bool LEFT_ENCODER_A,RIGHT_ENCODER_A, LEFT_ENCODER_B,RIGHT_ENCODER_B;
 /* MOTOR A */
 if (GPIO_Pin == GPIO_PIN_10)
 {
   // chương trình ngắt của chân 10

	 LEFT_ENCODER_A=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10);
	 state0=state0|LEFT_ENCODER_A;

	 state0=state0<<1;
	 LEFT_ENCODER_B=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11);
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

	 else if (GPIO_Pin == GPIO_PIN_11)
	 {
	   // chương trình ngắt của chân 11
		 LEFT_ENCODER_A=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10);
		 state1=state1|LEFT_ENCODER_A;

		 state1=state1<<1;
		 LEFT_ENCODER_B=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11);
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
	 else if (GPIO_Pin == GPIO_PIN_12)		 // LEFT CHANNEL B
	 {
		 // chương trình ngắt của chân 12

		 RIGHT_ENCODER_A=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12);
		 state2=state2|RIGHT_ENCODER_A;

		 state2=state2<<1;
		 RIGHT_ENCODER_B=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13);
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



	 else if (GPIO_Pin == GPIO_PIN_13)
	 {
		 // chương trình ngắt của chân 13

		 RIGHT_ENCODER_A=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12);
		 state3=state3|RIGHT_ENCODER_A;

		 state3=state3<<1;
		 RIGHT_ENCODER_B=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13);
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
	if(cnt==100) //1 cnt = 0.001s, default:80 = 0.8s
	{

		rads_left_velocity  = left_count*2*PI/(5376*0.001*cnt);
		rpm_left_velocity   = left_count*60/(5376*0.001*cnt);

		rads_right_velocity = right_count*2*PI/(5376*0.001*cnt);
		rpm_right_velocity  = right_count*60/(5376*0.001*cnt);

//		pos=previous_pos+right_count*360/5376;
//		previous_pos=pos;
		printf("%0.5f\n",rpm_right_velocity);
		left_count=0;
		right_count=0;
		cnt=0;
	}
}

void PID(float *SetPoint, float* ControlledVariable,float* PidOutput)
{
	// PWM mode has the range from 0 to 400.
	float HighLimit=400;
	static float ManipulatedVariable,ManipulatedVariableHat,uk,ui,previous_ui,CurrentError;

	// Calculate the error
	CurrentError=*SetPoint-*ControlledVariable;

	// Proportion
	uk=Kp*CurrentError;

	// Integration
	ui=previous_ui+Ki*CurrentError*0.1;
	ManipulatedVariable=ui+uk;

	if(ManipulatedVariable<HighLimit)
	{
		ManipulatedVariableHat=ManipulatedVariable;
		ResetError=0;
		*PidOutput=ManipulatedVariable;
	}
	if(ManipulatedVariable>HighLimit)
	{
		ManipulatedVariableHat=HighLimit;
		ResetError=ManipulatedVariableHat-ManipulatedVariable;
		AntiWindupError=Ki*CurrentError+ResetError*Kb;
		ui=previous_ui+Ki*AntiWindupError*0.8;
		*PidOutput=uk+ui;
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


