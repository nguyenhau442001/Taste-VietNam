/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#ifdef __cplusplus
 extern "C" {
#endif


void setup(void);
void loop(void);

#include "DCmotor.h"

#ifdef __cplusplus
}
#endif

// ROS LIBRARY
#include <ros.h>
#include <time.h>
#include "ros/duration.h"
#include <stdint.h>
#include <tf/tf.h>
#include "ros/time.h"
#include <std_msgs/String.h>  		 // library for /chatter topic
#include <sensor_msgs/Imu.h>  		 // library for /imu topic
#include <sensor_msgs/MagneticField.h>   // library for /magnetic topic
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>             // library for /odom topic
#include <turtlebot3_msgs/VersionInfo.h> // library for /firmware_version topic
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);

#endif /* MAINPP_H_ */
