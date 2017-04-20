#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from tms_ur_listener.msg import julius_msg
from std_msgs.msg import Bool
import os
import sys
import socket
import requests
import re
import subprocess
import shlex
import time

julius_path = '/usr/local/bin/julius'
jconf_path = '/home/pi/dictation-kit-v4.3.1-linux/tms.jconf'
julius = None
julius_socket = None


def invoke_julius():
    print 'INFO : invoke julius'
    args = julius_path + ' -C ' + jconf_path + ' -module '
    p = subprocess.Popen(
            shlex.split(args),
            stdin=None,
            stdout=None,
            stderr=None
        )
    print 'INFO : invoke julius complete.'
    print 'INFO : wait 2 seconds.'
    time.sleep(3.0)
    print 'INFO : invoke julius complete'
    return p


def kill_julius(julius):
    print 'INFO : terminate julius'
    julius.kill()
    while julius.poll() is None:
        print 'INFO : wait for 0.1 sec julius\' termination'
        time.sleep(0.1)
    print 'INFO : terminate julius complete'


def get_OS_PID(process):
    psef = 'ps -ef | grep ' + process + ' | grep -ve grep -vie python |head -1|awk \'{print($2)}\''
    if sys.version_info.major == 3:
        PID = str(subprocess.check_output(psef, shell=True), encoding='utf-8').rstrip ()
    else:
        PID = subprocess.check_output(psef, shell=True).rstrip ()
    return PID


def create_socket():
    print 'INFO : create a socket to connect julius'
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(('localhost', 10500))
    print 'INFO : create a socket to connect julius complete'
    return s


def delete_socket(s):
    print 'INFO : delete a socket'
    s.close()
    print 'INFO : delete a socket complete'
    return True


def invoke_julius_set():
    julius = invoke_julius()
    julius_socket = create_socket()
    sf = julius_socket.makefile('rb')
    return (julius, julius_socket, sf)

def power_callback(data):
    rospy.loginfo(data)
    if data.data == False:
        rospy.loginfo("DIE julius")
        kill_julius(julius)
        print "====================="
        time.sleep(2.0)
        print "====================="
        time.sleep(2.0)

def main():
    rospy.init_node("tms_ur_listener_client",anonymous=True)
    pub = rospy.Publisher('julius_msg',julius_msg,queue_size=10)
    rate = rospy.Rate(100)
    rospy.Subscriber("julius_power",Bool,power_callback)
    
    global julius
    global julius_socket
    julius, julius_socket, sf = invoke_julius_set()

    reResult = re.compile(u'WHYPO WORD="(\S*)" .* CM="(\d\.\d*)"')

    while not rospy.is_shutdown():
        if julius.poll() is not None:
            delete_socket(julius_socket)
            julius, julius_socket, sf = invoke_julius_set()
        else:
            line = sf.readline().decode('utf-8')
            print line
            tmp = reResult.search(line)
            if tmp:
                print tmp.group(1)
                msg = julius_msg()
                msg.data = tmp.group(1)
                msg.value = float(tmp.group(2))
                pub.publish(msg)
                #print tmp.group(1)
                # print line
                #if float(tmp.group(2)) > 0.8:
                 #   print 'WARN : DIE julius, call WATSON'
                  #  kill_julius(julius)
                   # delete_socket(julius_socket)
                    #print '====================================='
                    #time.sleep(2.0)
                    #print '====================================='
                    #time.sleep(2.0)
                    #print '====================================='
        rate.sleep()
    rospy.loginfo("exit")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException");
        kill_julius(julius)
        delete_socket(julius_socket)
        sys.exit(0)
