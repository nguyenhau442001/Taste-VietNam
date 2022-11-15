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
float CurrentError;
void PID(float *SetPoint, float* ControlledVariable,float* PidOutput)
{
	// PWM mode has the range from 0 to 400.
	float HighLimit=400,ManipulatedVariable,ManipulatedVariableHat,uk,ui;
	static float previous_ui;

	// Calculate the error
	CurrentError=*SetPoint-*ControlledVariable;

	// Proportion
	uk=Kp*CurrentError;

	// Integration
	ui=previous_ui+Ki*CurrentError*SAMPLE_TIME;
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
		ui=previous_ui+AntiWindupError*SAMPLE_TIME;
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

void SubcribeVelocityFromRos(float *linear_velocity,float *angular_velocity,float *left_velocity,float *right_velocity)
{

	// Calculate vel of each wheel
	*left_velocity  = ((2*(*linear_velocity)-(*angular_velocity)*WHEEL_SEPARATION))/2;  // unit: m/s
	*right_velocity = ((2*(*linear_velocity)+(*angular_velocity)*WHEEL_SEPARATION))/2;

	//v=omega.r => omega=v/r (rad/s)
	*left_velocity  = (*left_velocity)/WHEEL_RADIUS;
	*right_velocity = (*right_velocity)/WHEEL_RADIUS;

	// convert to RPM
	*left_velocity  = ((*left_velocity)*60)/(2*PI);
	*right_velocity = ((*right_velocity)*60)/(2*PI);

	// Determine the direction with the sign of value corresponding
	if((left_velocity>0)&&(right_velocity>0))
	{
		  // IN1,IN2 pin    (motor A)
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);

		  // IN3,IN4 pin	(motor B)
		  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);    // (0,1): < 0: forward. (1,0): >0 : reverse.
	      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
	}

	if((left_velocity<0)&&(right_velocity<0))
	{
		  // IN3,IN4 pin	(motor A)
		  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);    // (0,1): < 0: forward. (1,0): >0 : reverse.
	      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);

		  // IN1,IN2 pin   (motor B)
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
	}

	if((left_velocity>0)&&(right_velocity<0))
	{
		  // IN3,IN4 pin	(motor A)
		  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);    // (0,1): < 0: forward. (1,0): >0 : reverse.
	      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);

		  // IN1,IN2 pin   (motor B)
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
	}

	if((left_velocity<0)&&(right_velocity>0))
	{
		  // IN3,IN4 pin	(motor A)
		  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);    // (0,1): < 0: forward. (1,0): >0 : reverse.
	      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);

		  // IN1,IN2 pin   (motor B)
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
	}

}


