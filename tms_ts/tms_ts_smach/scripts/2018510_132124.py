#!/usr/bin/env python

import roslib; roslib.load_manifest('tms_ts_smach')
import rospy
import smach
import smach_ros

from smach_ros import ServiceState
from smach import Concurrence

from tms_msg_rp.srv import *
from tms_msg_ts.srv import *

def main():
    rospy.init_node('tms_ts_smach_executive1')

    sm_root = smach.StateMachine(['succeeded','aborted','preempted'])

    with sm_root:

        smach.StateMachine.add('move0',
                           ServiceState('rp_cmd',
                                        rp_cmd,
                                        request = rp_cmdRequest(9001, True, 2012, [6017])),
                           transitions={'succeeded':'control0'})

        smach.StateMachine.add('control0',
                           ServiceState('ts_state_control',
                                        ts_state_control,
                                        request = ts_state_controlRequest(0, 0, 0, 0, "")),
                           transitions={'succeeded':'succeeded', 'aborted':'aborted'})

    sis = smach_ros.IntrospectionServer('tms_ts_smach_test', sm_root, '/ROS_TMS')
    sis.start()

    outcome = sm_root.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
