#!/usr/bin/env python
# -*- coding:utf-8 -*-

## @file    afe.py
#  @brief   OEL blood stream sensor node
#  @author  Akio Shigekane
#  @date    2015.3.3

import rospy
import serial
import json
import pprint
import subprocess

from std_msgs.msg import Float32
from std_msgs.msg import Int32


PORT = "/dev/ttyUSB0"
KEYS = ("raw", "PRI")

dev = serial.Serial(
    # port=PORT,
    baudrate=115200,
    )


def main():
    print "Hello World"
    pub = {}
    pub[KEYS[0]] = rospy.Publisher('afe/blood_pressue', Float32, queue_size=10)
    pub[KEYS[1]] = rospy.Publisher('afe/period', Int32, queue_size=10)
    rospy.init_node("afe", anonymous=False)
    # print subprocess.check_output("rqt_plot /afe/blood_pressue &".split(" "))
    # print subprocess.check_output("rqt_plot /afe/period &".split(" "))
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        r.sleep()
        print "readling"
        in_js = dev.readline()
        print "in:", in_js
        in_dic = json.dumps(in_js)
        pprint.pprint(in_dic)
        for key, val in in_dic:
            pub[key].publish(val)
        pass


if __name__ == '__main__':
    main()
