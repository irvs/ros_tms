#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "kalman-Ndof.hpp"

Kalman *kalman;

ros::Publisher pos_pub, ninebot_pos_pub;
ros::Subscriber sub;

ros::Time last_time, now;

double X, Y, Z, qx, qy, qz, qw;
double preX, preY, preZ, preqx, preqy, preqz, preqw;
double kfX = 0;
double kfY = 0;
double kfZ = 0;
bool getInitRPY = true;
double init_roll, init_pitch, init_yaw;

void pozyx_kalman_callback(const geometry_msgs::PoseStamped &msg){
  // Store Prev Data
  preX = X;
  preY = Y;
  preZ = Z;
  preqx = qx;
  preqy = qy;
  preqz = qz;
  preqw = qw;

  // Store Present Data
  X = msg.pose.position.x;
  Y = msg.pose.position.y;
  Z = msg.pose.position.z;
  qx = msg.pose.orientation.x;
  qy = msg.pose.orientation.y;
  qz = msg.pose.orientation.z;
  qw = msg.pose.orientation.w;

  if(getInitRPY){
    tf::Quaternion init_q(qx,qy,qz,qw);
    tf::Matrix3x3 init_q_mat(init_q);
    init_q_mat.getRPY(init_roll, init_pitch, init_yaw);
    getInitRPY = false;
  }

  if(X == 0 && Y == 0 && Z == 0){
    X = preX;
    Y = preY;
    Z = preZ;
  }

  if(qx == 0 && qy == 0 && qz == 0 && qw == 0){
    qx = preqx;
    qy = preqy;
    qz = preqz;
    qw = preqw;
  }

  ROS_INFO("%f, %f, %f, %f, %f, %f, %f", X, Y, Z, qx, qy, qz, qw);

  // Calc attitude of Ninebot
  double ninebot_roll, ninebot_pitch, ninebot_yaw;
  tf::Quaternion temp_q(qx, qy, qz, qw);
  tf::Matrix3x3 temp_q_mat(temp_q);
  temp_q_mat.getRPY(ninebot_roll, ninebot_pitch, ninebot_yaw);

  ninebot_yaw -= init_yaw;

  tf::Matrix3x3 offset_mat;
  offset_mat.setEulerYPR(ninebot_yaw, ninebot_pitch, ninebot_roll);
  tf::Quaternion ninebot_quat;
  offset_mat.getRotation(ninebot_quat);

  // Apply Kalman Filter
  last_time = now;
  now = ros::Time::now();
  double delta_t = (now.sec + now.nsec * 0.000000001) - (last_time.sec + last_time.nsec * 0.000000001);
  double velX = (X - preX) / delta_t;
  double velY = (Y - preY) / delta_t;
  double velZ = (Z - preZ) / delta_t;

  static int count = 0;

  if(kalman == NULL) return;
  if(count == 0){
    count++;
    return;
  }else if(count == 1){
    // 初期化 (システム雑音，観測雑音，積分時間)
    kalman -> init(0.15, 0.7, 0.08);
    double C[6][6];
    memset(C, 0, sizeof(C));
    C[0][0] = C[1][1] = C[2][2] = 1;
    kalman -> setC((double *)&C);
    kalman -> setX(0, X);
    kalman -> setX(1, Y);
    kalman -> setX(2, Z);
    kalman -> setX(3, velX);
    kalman -> setX(4, velY);
    kalman -> setX(5, velZ);
    count++;
  }

  double A[3][6];
  memset(A, 0, sizeof(A));
  A[0][0] = A[1][1] = A[2][2] = 1;
  A[0][3] = A[1][4] = A[2][5] = delta_t;
  double obs[6] = {X, Y, Z, velX, velY, velZ};
  double input[3] = {0, 0, 0};

  kalman -> update(obs, input);

  kfX = kalman -> getX(0);
  kfY = kalman -> getX(1);
  kfZ = kalman -> getX(2);

  ROS_INFO("kfX = %f, kfY = %f, kfZ = %f", kfX, kfY, kfZ);

  // Publish "/pozyx" (PoseStamped)
  geometry_msgs::PoseStamped pos;
  pos.header.frame_id = msg.header.frame_id;
  pos.header.stamp = ros::Time::now();
  pos.pose.position.x = kfX;
  pos.pose.position.y = kfY;
  pos.pose.position.z = kfZ;
  pos.pose.orientation.x = qx;
  pos.pose.orientation.y = qy;
  pos.pose.orientation.z = qz;
  pos.pose.orientation.w = qw;
  pos_pub.publish(pos);

  // Publish "/ninebot_measured_pos" (Odometry)
  nav_msgs::Odometry odom;
  odom.header.frame_id = msg.header.frame_id;
  odom.header.stamp = ros::Time::now();
  odom.pose.pose.position.x = kfX;
  odom.pose.pose.position.y = kfY;
  odom.pose.pose.position.z = kfZ;
  odom.pose.pose.orientation.x = qx - ninebot_quat.getX();
  odom.pose.pose.orientation.y = qy - ninebot_quat.getY();
  odom.pose.pose.orientation.z = qz - ninebot_quat.getZ();
  odom.pose.pose.orientation.w = qw - ninebot_quat.getW();
  ninebot_pos_pub.publish(odom);

}

int main(int argc, char **argv){
  ros::init(argc, argv, "pozyx_kalman");
  ros::NodeHandle nh;
  kalman = new Kalman(6, 6, 3);
  now = ros::Time::now();
  pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("pozyx", 1000);
  ninebot_pos_pub = nh.advertise<nav_msgs::Odometry> ("ninebot_measured_pos", 1000);
  sub = nh.subscribe("/pozyx_rawdata", 1000, pozyx_kalman_callback);
  ros::spin();
  return 0;
}
