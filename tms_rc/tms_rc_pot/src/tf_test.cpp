//---------------------------------------------------------------
// @file    : portable_tf.cpp
// @author  : Watanabe Yuuta
// @version : Ver0.1.0 (since 2015.08.06)
// @date    : 2015
//---------------------------------------------------------------
#include "ros/ros.h"
#include "math.h"
#include "iostream"
#include "tf/transform_broadcaster.h"
#include "tms_msg_rc/robot_current_data.h"
#define PI 3.14159265359

// callback function
void plot_callback(const tms_msg_rc::robot_current_data &plot)
{
  // tf broadcaster
  static tf::TransformBroadcaster broadcaster;

  // robot
  geometry_msgs::Quaternion robot_quat;
  robot_quat = tf::createQuaternionMsgFromYaw(plot.fTheta);
  geometry_msgs::TransformStamped state;
  state.header.stamp = ros::Time::now();
  state.header.frame_id = "odom";
  state.child_frame_id = "portable";
  state.transform.translation.x = plot.fX;
  state.transform.translation.y = plot.fY;
  state.transform.translation.z = plot.fZ;
  state.transform.rotation = robot_quat;
  broadcaster.sendTransform(state);

  std::cout << "x " << plot.fX << " y " << plot.fY << " z " << plot.fZ << " theta " << plot.fTheta << std::endl;

  // laser
  geometry_msgs::Quaternion laser_quat;
  laser_quat = tf::createQuaternionMsgFromYaw(0.0);
  geometry_msgs::TransformStamped laser_state;
  laser_state.header.stamp = ros::Time::now();
  laser_state.header.frame_id = "portable";
  laser_state.child_frame_id = "portable_scan";
  laser_state.transform.translation.x = 0.0;
  laser_state.transform.translation.y = 0.0;
  laser_state.transform.translation.z = 1.0;
  laser_state.transform.rotation = laser_quat;
  broadcaster.sendTransform(laser_state);
}

// main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "portable_tf");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("virtual", 1000, &plot_callback);

  ros::spin();
  return 0;
}
