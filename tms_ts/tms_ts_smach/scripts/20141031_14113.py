#!/usr/bin/env python

import roslib; roslib.load_manifest('tms_ts_smach')
import rospy
import smach
import smach_ros

from smach_ros import ServiceState
from smach import Concurrence

from tms_msg_rp.srv import *

def main():
    rospy.init_node('tms_ts_smach_executive')

    sm_root = smach.StateMachine(['succeeded','aborted','preempted'])

    with sm_root:

        smach.StateMachine.add('move0',
                           ServiceState('rp_cmd',
                                        rp_cmd,
                                        request = rp_cmdRequest(9001, False, 2002, [7001])),
                           transitions={'succeeded':'grasp1'})

        smach.StateMachine.add('grasp1',
                           ServiceState('rp_cmd',
                                        rp_cmd,
                                        request = rp_cmdRequest(9002, False, 2002, [7001])),
                           transitions={'succeeded':'give2'})

        smach.StateMachine.add('give2',
                           ServiceState('rp_cmd',
                                        rp_cmd,
                                        request = rp_cmdRequest(9003, False, 2002, [1001])),
                           transitions={'succeeded':'move3'})

        smach.StateMachine.add('move3',
                           ServiceState('rp_cmd',
                                        rp_cmd,
                                        request = rp_cmdRequest(9001, False, 2002, [6099])),
                           transitions={'succeeded':'succeeded'})

    sis = smach_ros.IntrospectionServer('tms_ts_smach_test', sm_root, '/ROS_TMS')
    sis.start()

    outcome = sm_root.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
