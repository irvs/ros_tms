#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from tms_ur_listener.msg import julius_msg
from std_msgs.msg import Bool
from std_msgs.msg import String

class TmsUrListener():
    def __init__(self):
        rospy.init_node("tms_ur_listener")
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber("julius_msg",julius_msg,self.callback)
        self.power_pub = rospy.Publisher("julius_power",Bool,queue_size=10)
        self.speaker_pub = rospy.Publisher("speaker",String,queue_size=10)

    def callback(self, data):
        rospy.loginfo(data)
        if data.data == "スマートパル" and data.value > 0.8:
            rospy.loginfo("call smartpal")
            rospy.loginfo("kill julius!!!")
            msg = Bool()
            msg.data = False
            self.power_pub.publish(msg)
            speak = String()
            speak.data = "\sound1"
            self.speaker_pub.publish(speak)
        if data.data == "見守る君" and data.value > 0.8:
            rospy.loginfo("call mimamoru")
            rospy.loginfo("kill julius!!!")
            msg = Bool()
            msg.data = False
            self.power_pub.publish(msg)
            speak = String()
            speak.data = "\sound1"
            self.speaker_pub.publish(speak)
        if data.data == "TMS" and data.value > 0.8:
            rospy.loginfo("call TMS")
            rospy.loginfo("kill julius!!!")
            msg = Bool()
            msg.data = False
            self.power_pub.publish(msg)
            speak = String()
            speak.data = "\sound1"
            self.speaker_pub.publish(speak)

    def shutdown(self):
        rospy.loginfo("Stopping the node")

if __name__ == '__main__':
    try:
        TmsUrListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("tms_ur_listener node terminated.")
