#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from tms_ss_kinect_v2.msg import SkeletonArray
from tms_ss_kinect_v2.msg import pose_setting
from tms_ss_kinect_v2.srv import ConvertToJointAngles


class SkeletonCnoidBridge:
    def callback(self, msg):
        # for skeleton in [msg.data[0]]:
        for skeleton in msg.data:
            rospy.loginfo('Send skeleton to choreonoid: ID -> {0}'
                          .format(skeleton.user_id))
            rospy.wait_for_service('convert_to_jointangles')
            try:
                convert_to_jointangles = \
                    rospy.ServiceProxy('convert_to_jointangles',
                                       ConvertToJointAngles)
                respl = convert_to_jointangles(skeleton)
                respl.joint_angles.user_id = skeleton.user_id
                self.pub.publish(respl.joint_angles)
            except rospy.ServiceException, e:
                print('Service call failed: {0}'.format(e))

    def run(self):
        rospy.init_node('skeleton_cnoid_bridge')
        self.pub = rospy.Publisher('skeleton_pose_set',
                                   pose_setting, queue_size=10)
        self.sub = rospy.Subscriber("integrated_skeleton_stream",
                                    SkeletonArray, self.callback)
        self.freqency = rospy.Rate(10)
        rospy.spin()

if __name__ == '__main__':
    try:
        obj = SkeletonCnoidBridge()
        obj.run()

    except rospy.ROSInterruptException:
        pass
