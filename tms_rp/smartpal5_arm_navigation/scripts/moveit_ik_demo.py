#!/usr/bin/env python

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItFKDemo:
    def __init__(self):
        # Initialize the move_group API and node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_ik_demo', anonymous=True)

        # Use the groups of SmartPal5
        arm_left = moveit_commander.MoveGroupCommander('arm_left')
        gripper_left = moveit_commander.MoveGroupCommander('gripper_left')
        end_effector_link = arm_left.get_end_effector_link() # leftGripper_link
        reference_frame = 'start_position'

        # Set a goal joint tolerance
        arm_left.set_goal_joint_tolerance(0.001)
        gripper_left.set_goal_joint_tolerance(0.001)

        # Set the option related IK solution
        arm_left.allow_replanning(True)

        # Use the pose stored in the SRDF file
        # 1. Set the target pose
        # 2. Plan a trajectory
        # 3. Execute the planned trajectory
        arm_left.set_named_target('arm_left_init')
        arm_left.go()
        rospy.sleep(1)
         
        gripper_left.set_named_target('gripper_left_init')
        gripper_left.go()
        rospy.sleep(1)

        # Use the joint pose with IK
        # 1. Set the target pose for IK
        # 2. Plan a trajectory
        # 3. Execute the planned trajectory
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.370748751767
        target_pose.pose.position.y = 0.420162677108
        target_pose.pose.position.z = 0.840162164512
        target_pose.pose.orientation.x = 0.0246617763278
        target_pose.pose.orientation.y = 0.024823036458
        target_pose.pose.orientation.z = -0.706731080872
        target_pose.pose.orientation.w = 0.706616439789

        arm_left.set_start_state_to_current_state()
        arm_left.set_pose_target(target_pose, end_effector_link)
        traj = arm_left.plan()
        arm_left.execute(traj)
        rospy.sleep(1)

        # Shift the target pose (0,1,2,3,4,5 = x,y,z,r,p,y)
        arm_left.shift_pose_target(1, -0.1, end_effector_link)
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
