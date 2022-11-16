/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_tim.h>

/********** Declare struct **********/
PID_TypeDef uPID;
Error_TypeDef err;

ros::NodeHandle nh;

/********** Declare publisher **********/
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello Taste VN!";

/********** Declare subscriber **********/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

float PidOut[2];

/********** Extern variables **********/
extern float SetPointAngularVelocity[2]={17.0,17.0};
extern float ActualAngularVelocity[2]  ;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
	SubcribeVelocityFromRos(0.01,0);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(cmd_vel_sub);


}

void loop(void)
{

  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);

  ReadEncoder();
  ComputeVelocity();
  SubcribeVelocityFromRos(0.01,0);
  PID(&uPID,&err,0.229,15.3,22.222,SetPointAngularVelocity[0],ActualAngularVelocity[0],&PidOut[0]);
  PID(&uPID,&err,0.229,15.3,22.222,SetPointAngularVelocity[1],ActualAngularVelocity[1],&PidOut[1]);
    HAL_Delay(1000*SAMPLE_TIME);
  	  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,fabs(round(PidOut[0])));
  	  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,fabs(round(PidOut[1])));

  str_msg.data = hello;
  chatter.publish(&str_msg);
  nh.spinOnce();
//
//  HAL_Delay(100);
}

