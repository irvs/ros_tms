#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import tf
import math

from nav_msgs.msg      import Odometry
from geometry_msgs.msg import Twist


ANGULAR_MAX = 0.2  #最終的に出力される角速度の絶対値の最大値
ANGULAR_K   = 0.5  #曲率を維持する際のスケール変更の係数(マジックナンバー)

LOW_ALPHA   = 0.9  #台形加速のためにローパスフィルタを使用する際のアルファ
ACCELE_CW   = 0.03 #台形加速の角加速度(100Hzの１ループ中での角速度の最大増減分)


class SmootherClass:
  def __init__(self):

    rospy.init_node('ninebot_smoother', anonymous=True)
    rospy.on_shutdown(self.shutdown)

    self.now_twist    = Twist()
    self.serial_twist = Twist()
    self.twist_pub    = rospy.Publisher('serial_twist', Twist, queue_size=100)

    rospy.Subscriber('cmd_vel', Twist, self.callback_twist, queue_size=10)
    rospy.Subscriber('odom', Odometry, self.callback_odom, queue_size=10)

    p_tgt_vw = 0.0
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():

      serial_twist = self.serial_twist
      tgt_vw = serial_twist.angular.z
      
      if   ((tgt_vw > 0) and (tgt_vw > p_tgt_vw + ACCELE_CW)): tgt_vw = p_tgt_vw + ACCELE_CW
      elif ((tgt_vw < 0) and (tgt_vw < p_tgt_vw - ACCELE_CW)): tgt_vw = p_tgt_vw - ACCELE_CW
      #tgt_vw = p_tgt_vw * LOW_ALPHA +  tgt_vw * (1.0 - LOW_ALPHA)       
      p_tgt_vw = tgt_vw

      serial_twist.angular.z = self.limit(tgt_vw, -ANGULAR_MAX, ANGULAR_MAX)
      self.twist_pub.publish(serial_twist)
      rate.sleep()

  def callback_twist(self, data):
    now_twist = self.now_twist
    
    # if   data.angular.z > 0: data.angular.z =  math.sqrt(10 * math.fabs(data.angular.z)) * 0.1
    # elif data.angular.z < 0: data.angular.z = -math.sqrt(10 * math.fabs(data.angular.z)) * 0.1

    #想定の方向への速度が一定速以上出ている場合は、なるべく曲率を維持した角速度を計算する
    if (math.fabs(now_twist.linear.x) > 0.08) and (now_twist.linear.x * data.linear.x > 0.0):
      data.angular.z = data.angular.z * ANGULAR_K * math.fabs(now_twist.linear.x / data.linear.x)
 
    #想定と逆方向への速度が一定速以上出ている場合は、音を鳴らして警告する
    if (math.fabs(now_twist.linear.x) > 0.08) and (data.linear.x < 0.0):
      data.linear.z = 1.0

    self.serial_twist = data

  def callback_odom(self, data):
    self.now_twist = data.twist.twist

  def limit(self, value, min_value, max_value):
    return_value = value
    if max_value < return_value:
      return_value = max_value
    if return_value < min_value:
      return_value = min_value
    return return_value

  def shutdown(self):
    self.twist_pub.publish(Twist())

if __name__ == '__main__':
  try:
    SmootherClass()
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo("ninebot_smoother node finished.")
