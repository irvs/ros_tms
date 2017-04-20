#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import subprocess

def jtalk(t):
    open_jtalk=['open_jtalk']
    mech=['-x','/var/lib/mecab/dic/open-jtalk/naist-jdic']
    htsvoice=['-m','/usr/share/hts-voice/mei/mei_normal.htsvoice']
    speed=['-r','1.0']
    outwav=['-ow','open_jtalk.wav']
    cmd=open_jtalk+mech+htsvoice+speed+outwav
    c = subprocess.Popen(cmd,stdin=subprocess.PIPE)
    c.stdin.write(t)
    c.stdin.close()
    c.wait()
    aplay = ['aplay','-q','open_jtalk.wav']
    wr = subprocess.Popen(aplay)

def callback(data):
    rospy.loginfo(data)
    if data.data[0]=='\\':
        aplay = ['aplay','-q',data.data[1:]+'.wav']
        wr = subprocess.Popen(aplay)
    else:
        jtalk(data.data)

def main():
    rospy.init_node("tms_ur_speaker",anonymous=True)
    rospy.Subscriber("speaker",String,callback)
    rospy.loginfo("ready")
    rospy.spin()
    rospy.loginfo("exit")
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException");
        sys.exit(0)
