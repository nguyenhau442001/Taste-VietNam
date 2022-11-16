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
ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello Taste VN!";

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

float LeftPidOut,RightPidOut;
float left_vel=17,right_vel=17.0;
float v=0.0,omega=0.0;
extern float rpm_left_velocity,rpm_right_velocity;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
	SubcribeVelocityFromRos(cmd_vel_msg.linear.x,cmd_vel_msg.angular.z,&left_vel,&right_vel);
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

  PID(&left_vel,&rpm_left_velocity,&LeftPidOut);
//    PID(&right_vel,&rpm_right_velocity,&RightPidOut);
  HAL_Delay(1000*SAMPLE_TIME);
	  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,fabs(round(LeftPidOut)));
//	  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,fabs(round(RightPidOut)));

//	  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,fabs(round(200)));
  str_msg.data = hello;
  chatter.publish(&str_msg);
  nh.spinOnce();
//
//  HAL_Delay(100);
}

