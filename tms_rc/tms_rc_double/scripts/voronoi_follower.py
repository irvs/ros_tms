#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Pose2D, Twist
from tms_msg_rc.srv import rc_robot_control, rc_robot_controlResponse
from tms_msg_db.srv import TmsdbGetData, TmsdbGetDataRequest

import datetime
import pymongo
from math import sin, cos, atan2, pi, radians, degrees, sqrt
pub = rospy.Publisher("cmd_vel_mux/input/keyop",Twist,queue_size = 10)

GOAL = None

def main():
    global GOAL
    print "Double_voronoi_follower"
    rospy.init_node ('double_voronoi_follower')

    rospy.wait_for_service('/tms_db_reader')

    service = rospy.Service(
        "double_goal_pose" , rc_robot_control, goalPoseCallBack)
    
    r = rospy.rate(10)
    while not rospy.is_shutdown():
        if None == GOAL:
            continue
        KPang = 0.2
        KDang = 0
        KPdist = 0.1
        KDdist = 0
        ARV_DIST = 0.25

        pose = getCurrentPose()
        errorX = GOAL.x - pose.x
        errorY = GOAL.y - pose.y
        targetT = atan2(errorY, errorX)

        errorNX = errorX * cos(-pose.theta) - errorY * sin(-pose.theta)
        errorNT = normalizeAng(targetT - pose.theta)

        tmp_spd = limit(KPdist * errorNX, 100, -100)
        tmp_turn = limit(KPang * degrees(errorNT), 30, -30)

        twist = Twist()
        distance = sqrt(errorX ** 2 + errorY **2)
        rospy.loginfo("dist:{0}".format(distance))
        rospy.loginfo("psd:{0}" "turn:{1}".format(tmp_spd, tmp_turn))
        if distance <= ARV_DIST:
            twist.angular.z = 0
            twist.linear.x = 0
            GOAL = None
        else:
            twist.angular.z = radians(tmp_turn)
            twist.linear.x = tmp_spd
        pub.publish(twist)
        r.sleep()

def goalPoseCallBack(req):
    
    global GOAL
    GOAL = Pose2D()
    GOAL.x = req.arg[0]
    GOAL.y = req.arg[1]
    GOAL.theta = radians(req.arg[2])

    return rc_robot_controlResponse()

def getCurrentPose():
    pose = Pose2D()
    db_req = TmsdbGetDataRequest()
    db_req.tmsdb.id = 2012
    db_req.tmsdb.sensor = 3001
    try:
        srv_client = rospy.ServiceProxy("/tms_db_reader", TmsdbGetData)

        res = srv_client(db_req)
        if 0 == len(res.tmsdb):
            return pose
        pose.x = res.tmsdb[0].x
        pose.y = res.tmsdb[0].y
        pose.theta = res.tmsdb.ry

    except rospy.ServiceException as e:
        print "Service call failed: %s" %e
    
    return pose



def normalizeAng(rad):
    while rad > pi:   # 角度を-180°~180°(-π~π)の範囲に合わせる
        rad = rad - (2 * pi)
    while rad < -pi:
        rad = rad + (2 * pi)
    return rad


def limit(val, maxn, minn):
    return max(min(maxn, val), minn)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
