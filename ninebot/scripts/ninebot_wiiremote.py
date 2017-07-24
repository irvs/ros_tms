#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import time

from sensor_msgs.msg   import Joy
from geometry_msgs.msg import Twist


ANGULAR_MAX    = 0.1
WIIREMOTE_PLAY = 0.08     


class WiiRemoteClass:
  def __init__(self):
    rospy.init_node('ninebot_wiiremote', anonymous=True)
    rospy.on_shutdown(self.shutdown)
    topic_name = rospy.get_param('~cmd_vel_topic', 'serial_twist')
    
    self.twist_pub = rospy.Publisher(topic_name, Twist, queue_size=1)
    self.twist = Twist()
    self.twist.angular.z = 0
    self.twist_pub.publish(self.twist)

    rospy.Subscriber('/wiimote/nunchuk', Joy, self.callback, queue_size=1)
       
  def callback(self, data):

    value = self.limit(data.axes[0], -1.0, 1.0)
    
    if(abs(value) <  WIIREMOTE_PLAY): 
      value = 0
    else:                             
      value = value * ANGULAR_MAX

    self.twist.angular.z = value
    self.twist_pub.publish(self.twist)
  
  def limit(self, value, min_value, max_value):
    return_value = value
    if max_value < return_value:
      return_value = max_value
    if return_value < min_value:
      return_value = min_value
    return return_value

  def shutdown(self):
    try:
      self.twist.angular.z = 0
      self.twist_pub.publish(self.twist)
    except: pass

if __name__ == '__main__':
  try:
    WiiRemoteClass()
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo("ninebot_wiiremote node finished.")