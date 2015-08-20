#!/usr/bin/env python

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand

class MoveItFKDemo:
    def __init__(self):
        # Initialize the move_group API and node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_fk_demo', anonymous=True)

        # Use the groups of SmartPal5
        arm_left = moveit_commander.MoveGroupCommander('arm_left')
        gripper_left = moveit_commander.MoveGroupCommander('gripper_left')
                
        # Set a goal joint tolerance
        arm_left.set_goal_joint_tolerance(0.001)
        gripper_left.set_goal_joint_tolerance(0.001)

        # Use the pose stored in the SRDF file
        # 1. Set the target pose
        # 2. Plan a trajectory
        # 3. Execute the planned trajectory
        arm_left.set_named_target('arm_left_init')
        traj = arm_left.plan()
        arm_left.execute(traj)
        rospy.sleep(1)
         
        gripper_left.set_named_target('gripper_left_init')
        gripper_left.go()
        rospy.sleep(1)

        # Use the joint positions with FK
        # 1. Set the target joint values
        # 2. Plan a trajectory
        # 3. Execute the planned trajectory
        joint_positions = [0.0, 0.07, 0.0, 1.5707, 0.0, 0.0, 0.0]
        arm_left.set_joint_value_target(joint_positions)
        arm_left.go()
        rospy.sleep(1)
         
        gripper_left.set_named_target('gripper_left_open')
        gripper_left.go()
        rospy.sleep(1)
                 
        # Return
        arm_left.set_named_target('arm_left_init')
        traj = arm_left.plan()
        arm_left.execute(traj)
        rospy.sleep(1)
         
        gripper_left.set_named_target('gripper_left_init')
        gripper_left.go()
        rospy.sleep(1)

        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()

        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItFKDemo()
    except rospy.ROSInterruptException:
        pass
