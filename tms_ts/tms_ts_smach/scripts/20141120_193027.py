#!/usr/bin/env python

import roslib
roslib.load_manifest('tms_ts_smach')
import rospy
import smach
import smach_ros

from smach_ros import ServiceState
from smach import Concurrence

from tms_msg_rp.srv import *
from tms_msg_ts.srv import *


def smc0():
    smc0 = smach.Concurrence(outcomes=['succeeded', 'aborted'],
                             default_outcome='aborted',
                             outcome_map={
                             'succeeded': {'0random_move': 'succeeded', '1sensing': 'succeeded'}},
                             child_termination_cb=lambda arg: True)

    with smc0:
        smach.Concurrence.add('0random_move',
                              ServiceState('rp_cmd',
                                           rp_cmd,
                                           request=rp_cmdRequest(9006, True, 2005, [0])))
        smach.Concurrence.add('1sensing',
                              ServiceState('rp_cmd',
                                           rp_cmd,
                                           request=rp_cmdRequest(9007, True, 2005, [0])))
    return smc0


def main():
    rospy.init_node('tms_ts_smach_executive1')

    sm_root = smach.StateMachine(['succeeded', 'aborted', 'preempted'])

    with sm_root:

        smach.StateMachine.add(
            'smc0', smc0(), transitions={'succeeded': 'control0'})

        smach.StateMachine.add('control0',
                               ServiceState('ts_state_control',
                                            ts_state_control,
                                            request=ts_state_controlRequest(
                                            0, 0, 0, 2, "")),
                               transitions={'succeeded': 'move1', 'aborted': 'aborted'})

        smach.StateMachine.add('move1',
                               ServiceState('rp_cmd',
                                            rp_cmd,
                                            request=rp_cmdRequest(
                                            9001, True, 2005, [0])),
                               transitions={'succeeded': 'control1'})

        smach.StateMachine.add('control1',
                               ServiceState('ts_state_control',
                                            ts_state_control,
                                            request=ts_state_controlRequest(
                                            0, 0, 0, 0, "")),
                               transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    sis = smach_ros.IntrospectionServer(
        'tms_ts_smach_test', sm_root, '/ROS_TMS')
    sis.start()

    outcome = sm_root.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
