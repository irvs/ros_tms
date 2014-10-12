#!/usr/bin/env python

import roslib; roslib.load_manifest('tms_ts_smach')
import rospy
import smach
import smach_ros

from smach_ros import ServiceState
from tms_msg_rp.srv import *


class move0(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state move0')


class grasp1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state grasp1')


class actioin2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state actioin2')


class give3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state give3')


def main():
    rospy.init_node('tms_ts_smach_test')

    sm_root = smach.StateMachine(['succeeded','aborted','preempted'])

    with sm_root:

        smach.StateMachine.add('move0',
                           ServiceState('rp_cmd',
                                        rp_cmd,
                                        request = rp_cmdRequest(9001, 2002, [6011])),
                           transitions={'succeeded':'grasp1'})

        smach.StateMachine.add('grasp1',
                           ServiceState('rp_cmd',
                                        rp_cmd,
                                        request = rp_cmdRequest(9002, 2002, [7001])),
                           transitions={'succeeded':'actioin2'})

        smach.StateMachine.add('actioin2',
                           ServiceState('rp_cmd',
                                        rp_cmd,
                                        request = rp_cmdRequest(9004, 2002, [2002])),
                           transitions={'succeeded':'give3'})

        smach.StateMachine.add('give3',
                           ServiceState('rp_cmd',
                                        rp_cmd,
                                        request = rp_cmdRequest(9003, 2002, [1001])),
                           transitions={'succeeded':'succeeded'})

    sis = smach_ros.IntrospectionServer('server_name', sm_root, '/SM_ROOT')
    sis.start()

    outcome = sm_root.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
