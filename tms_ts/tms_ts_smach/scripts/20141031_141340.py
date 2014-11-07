#!/usr/bin/env python

import roslib; roslib.load_manifest('tms_ts_smach')
import rospy
import smach
import smach_ros

from smach_ros import ServiceState
from smach import Concurrence

from tms_msg_rp.srv import *

def smc0():
    smc0 = smach.Concurrence( outcomes=['succeeded', 'aborted', 'preempted'],
                            default_outcome = 'aborted',
                            outcome_map = {'succeeded': {'0random_move':'succeeded'},
                                           'aborted': {'0random_move':'aborted'},
                                           'preempted': {'0random_move':'preempted'}},
                            child_termination_cb = lambda arg: True )

    with smc0:
        smach.Concurrence.add('0random_move',
                           ServiceState('rp_cmd',
                                        rp_cmd,
                                        request = rp_cmdRequest(9006, False, 2005, [0])))
        smach.Concurrence.add('1sensing',
                           ServiceState('rp_cmd',
                                        rp_cmd,
                                        request = rp_cmdRequest(9007, False, 2005, [0])))
    return smc0

def main():
    rospy.init_node('tms_ts_smach_executive')

    sm_root = smach.StateMachine(['succeeded','aborted','preempted'])

    with sm_root:

        smach.StateMachine.add('smc0', smc0(), transitions={'succeeded':'move1', 'aborted':'aborted', 'preempted':'preempted'})

        smach.StateMachine.add('move1',
                           ServiceState('rp_cmd',
                                        rp_cmd,
                                        request = rp_cmdRequest(9001, False, 2005, [1001])),
                           transitions={'succeeded':'succeeded'})

    sis = smach_ros.IntrospectionServer('tms_ts_smach_test', sm_root, '/ROS_TMS')
    sis.start()

    outcome = sm_root.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
