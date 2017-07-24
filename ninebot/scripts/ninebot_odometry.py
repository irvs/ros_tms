#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import tf
import math

from nav_msgs.msg      import Odometry
from sensor_msgs.msg   import Imu
from ninebot.msg       import nucleo_serial


RADIUS    = 0.123  #タイヤの半径(人が乗った場合 ： 0.123, 手で押す場合 : 0.129)
TREAD     = 0.485  #タイヤ間距離
R_PULSE   = 4000   #エンコーダが一周した際のエンコーダ値（パルス数×4）

K_ENCODER = 2.0 * math.pi * RADIUS / R_PULSE


class OdometryClass:
  def __init__(self):

    self.odom_frame = rospy.get_param('~odom_frame_id', 'odom1')
    self.base_frame = rospy.get_param('~base_frame_id', 'base_footprint1')

    self.delta_r    = self.delta_l    = 0.0
    self.position_x = self.position_y = 0.0
    self.pose_yaw   = self.vel_theta  = 0.0 

    self.odom_pub    = rospy.Publisher('odom', Odometry, queue_size=1000)
    self.broadcaster = tf.TransformBroadcaster()

    rospy.Subscriber('nucleo_serial', nucleo_serial, self.callback_serial, queue_size=1)

  def callback_serial(self, data):
    if data.delta_t == 0: return

    self.delta_r = K_ENCODER * data.raw_delta_r
    self.delta_l = K_ENCODER * data.raw_delta_l
    vel_theta = (self.delta_r - self.delta_l) / (data.delta_t * TREAD)
    
    self.simple_integral(self.delta_r, self.delta_l)
    # self.precision_integral(self.delta_r, self.delta_l, data.delta_t, vel_theta)

    self.pose_yaw += vel_theta * data.delta_t
    while self.pose_yaw >  math.pi: self.pose_yaw -= 2.0 * math.pi
    while self.pose_yaw < -math.pi: self.pose_yaw += 2.0 * math.pi

    quate = tf.transformations.quaternion_from_euler(0.0, 0.0, self.pose_yaw)
    self.broadcaster.sendTransform( \
      (self.position_x, self.position_y, 0.0),  \
      (quate[0], quate[1], quate[2], quate[3]), \
      rospy.Time.now(), self.base_frame, self.odom_frame)
    odom                         = Odometry()
    odom.header.stamp            = rospy.Time.now()
    odom.header.frame_id         = self.odom_frame
    odom.child_frame_id          = self.base_frame
    odom.pose.pose.position.x    = self.position_x
    odom.pose.pose.position.y    = self.position_y
    odom.pose.pose.orientation.x = quate[0]
    odom.pose.pose.orientation.y = quate[1]
    odom.pose.pose.orientation.z = quate[2]
    odom.pose.pose.orientation.w = quate[3]
    odom.twist.twist.linear.x    = (self.delta_r + self.delta_l) / (data.delta_t * 2.0)
    odom.twist.twist.angular.z   = vel_theta
    self.odom_pub.publish(odom)

  def simple_integral(self, dr, dl):
    self.position_x += math.cos(self.pose_yaw) * (dr + dl) * 0.5
    self.position_y += math.sin(self.pose_yaw) * (dr + dl) * 0.5
  
  def precision_integral(self, dr, dl, dt, vel_theta):
    dth = vel_theta * dt
    if dth == 0:
      dx = ((dr + dl) * math.sin(dth * 0.5) * math.cos(self.pose_yaw + (dth * 0.5))) / dth
      dy = ((dr + dl) * math.sin(dth * 0.5) * math.sin(self.pose_yaw + (dth * 0.5))) / dth
    else:
      dx = (dr + dl) * math.cos(self.pose_yaw + (dth * 0.5)) * 0.5
      dy = (dr + dl) * math.sin(self.pose_yaw + (dth * 0.5)) * 0.5           
    self.position_x += dx
    self.position_y += dy

if __name__ == '__main__':
  rospy.init_node('ninebot_odometry', anonymous=True)
  try:
    try:
      RADIUS = float(rospy.get_param('~wheel_radius', RADIUS))
      K_ENCODER = 2.0 * math.pi * RADIUS / R_PULSE
    except: pass
    OdometryClass()
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo("ninebot_odometry node finished.")