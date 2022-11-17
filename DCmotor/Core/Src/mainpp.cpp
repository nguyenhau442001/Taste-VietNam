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
extern PID_TypeDef uPID;
Error_TypeDef err;
extern float SetPointAngularVelocity[2]={17.0,17.0};
extern float ActualAngularVelocity[2]  ;
float PidOut[2];
ros::NodeHandle nh;

/********** Declare publisher **********/
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello Taste VN!";

/********** Declare subscriber **********/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);



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

  PID2Motor(&uPID,&err,0.229,15.3,22.222,0.05,SetPointAngularVelocity,ActualAngularVelocity,PidOut);


  str_msg.data = hello;
  chatter.publish(&str_msg);
  nh.spinOnce();

}

