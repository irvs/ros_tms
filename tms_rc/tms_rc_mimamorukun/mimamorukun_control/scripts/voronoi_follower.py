#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Pose2D, Twist
from tms_msg_rc.srv import rc_robot_control
from tms_msg_db.srv import TmsdbGetData, TmsdbGetDataRequest

import datetime
import pymongo
from math import sin, cos, atan2

def main():
    print ("\x1b[32mHello World\x1b[39m")
    rospy.init_node('wheelchair_voronoi_follower')

    print ("finding DB service server")
    rospy.wait_for_service('/tms_db_reader')
    print ("found DB service server")

    service = rospy.Service("mkun_goal_pose", rc_robot_control, goalPoseCallback)
    pub = rospy.Publisher("mimamorukun/cmd_vel", Twist, queue_size=10)

    # req = TmsdbGetDataRequest()
    # req.tmsdb.id = 2007     # wheelchair
    # req.tmsdb.sensor = 3001 # vicon
    # r = rospy.Rate(4)
    while True:
        pass
        # try:
        #     srv_client = rospy.ServiceProxy("/tms_db_reader",
        #                                     TmsdbGetData)
        #     res = srv_client(req)
        #     if 0 < len(res.tmsdb):
        #         print datetime.datetime.now(),
        #         print"x:%4f  y:%4f yaw:%4f" % (res.tmsdb[0].x, res.tmsdb[0].y, res.tmsdb[0].ry)
        #         curr_pose = Pose2D()
        #         curr_pose.x = res.tmsdb[0].x
        #         curr_pose.y = res.tmsdb[0].y
        #         curr_pose.theta = res.tmsdb[0].ry
        # except rospy.ServiceException, e:
        #     print ("Service call failed: %s" % e)
        # r.sleep()


def goalPoseCallback(req):
    print "Received the service call!"
    rospy.loginfo("Received the service call!")
    rospy.loginfo(req)
    KPang = 1
    KDang = 0
    KPdist = 1
    KDdist = 0

    pose = getCurrentPose()

    goal = Pose2D()
    goal.x = req.arg[0]
    goal.y = req.arg[1]
    goal.theta = req.arg[2]/180.0*3.141592

    errorX = goal.x - pose.x;
    errorY = goal.y - pose.y;
    targetT = atan2(errorY, errorX);
    # theta = this->pos_vicon.theta;
    errorNX = errorX * cos(-pose.theta) - errorY * sin(-pose.theta);
    errorNT = nomalizeAng(targetT - theta);

    tmp_spd = Limit(KPdist * errorNX, 100, -100)
    tmp_turn = Limit(KPang * Rad2Deg(errorNT), 30, -30)
    print "spd:{0} turn:{1}".format(mp_spd, tmp_turn)

    twist = Twist()
    distance = sqrt(sqr(errorX) + sqr(errorY))
    if distance <= ARV_DIST:
        twist.angular.z = 0
        twist.linear.x = 0
    else:
        twist.angular.z = Deg2Rad(tmp_turn)
        twist.linear.x = tmp_spd
    pub.publish(twist)

def getCurrentPose():
    pose = Pose2D()
    db_req = TmsdbGetDataRequest()
    db_req.tmsdb.id = 2007     # wheelchair
    db_req.tmsdb.sensor = 3001 # vicon
    try:
        srv_client = rospy.ServiceProxy("/tms_db_reader",
                                        TmsdbGetData)
        res = srv_client(db_req)
        if 0 == len(res.tmsdb):
            return pose     # failed
        print datetime.datetime.now(),
        print"x:%4f  y:%4f yaw:%4f" % (res.tmsdb[0].x, res.tmsdb[0].y, res.tmsdb[0].ry)
        pose.x = res.tmsdb[0].x
        pose.y = res.tmsdb[0].y
        pose.theta = res.tmsdb[0].ry
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)
    return pose

if __name__ == '__main__':
    main()
