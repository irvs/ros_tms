#!/usr/bin/env python
# -*- coding:utf-8 -*-

## @file    afe.py
#  @brief   OEL blood stream sensor node
#  @author  Akio Shigekane
#  @date    2015.3.3

import rospy
import serial
import json
import traceback
import pprint
import subprocess

from std_msgs.msg import Int32

PORT = "/dev/ttyUSB0"
KEYS = ("Raw", "HBR")

dev = serial.Serial(
    port=PORT,
    baudrate=115200,
    )


def main():
    print "Hello World"
    pub = {}
    pub[KEYS[0]] = rospy.Publisher('afe/blood_pressue', Int32, queue_size=10)
    pub[KEYS[1]] = rospy.Publisher('afe/hbr', Int32, queue_size=10)
    rospy.init_node("afe", anonymous=False)
    # print subprocess.check_output("rqt_plot /afe/blood_pressue &".split(" "))
    # print subprocess.check_output("rqt_plot /afe/period &".split(" "))
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        r.sleep()
        print ""
        in_js = dev.readline().replace('\n\r','')
        print type(in_js),"   in:", in_js
        in_dic = {}
        try:
            in_dic = json.loads(in_js)
        except:
            print traceback.format_exc()
        pprint.pprint(in_dic)
        for key, val in in_dic.items():
            pub[key].publish(val)
        pass


if __name__ == '__main__':
    main()
