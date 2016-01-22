#!/usr/bin/env python

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy
from tms_msg_db.msg import TmsdbStamped, Tmsdb
from tms_msg_db.srv import *
from tms_msg_rp.srv import *

GROUP_NAME_ARM = 'l_arm'
GROUP_NAME_GRIPPER = 'l_gripper'

GRIPPER_FRAME = 'l_end_effector_link'
GRIPPER_JOINT_NAMES = ['l_gripper_thumb_joint']
GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'world_link'

NEUTRAL = 0
ARM_NEUTRAL = 1
ARM_GRASPING = 2
ARM_GRASPING2 = 3
ARM_GIVE = 4
ARM_GIVE_END = 5
ARM_SERVING = 6

GRIPPER_NEUTRAL = 100
GRIPPER_OPEN = 101
GRIPPER_CLOSE = 102

# ARM_NEUTRAL_VAL = [0.0, -0.17,0.0,0.0,0.0,0.0,0.0]
# ARM_GRASPING_VAL = [0.84,-1.31,-0.73,0.65,1.13,-0.24,0.55]
# ARM_GRASPING2_VAL = [0.0, -0.2,0.0,0.0,0.0,0.0,0.0]
# ARM_GIVE_VAL = [0.65,-0.1,0.0,0.93,0.0,0.0,0.0]
# ARM_GIVE_END_VAL = [0.65,-0.5,0.0,0.93,0.0,0.0,0.0]
#
# GRIPPER_NEUTRAL_VAL = [-0.2]
# GRIPPER_OPEN_VAL = [-1.0]
# GRIPPER_CLOSE_VAL = [-0.7]

class SubTaskArmMove:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('subtask_arm_move')
        rospy.on_shutdown(self.shutdown)

        self.arm_move_srv = rospy.Service('subtask_arm_move', rp_arm_move, self.ArmMoveSrvCallback)

    def ArmMoveSrvCallback(self, req):
        rospy.loginfo("Received the service call!")
        rospy.loginfo(req)

        arm = MoveGroupCommander(GROUP_NAME_ARM)
        gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)

        scene = PlanningSceneInterface()

        arm.set_goal_position_tolerance(0.05)
        arm.set_goal_orientation_tolerance(0.1)

        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)

        # Set the right arm reference frame
        arm.set_pose_reference_frame(REFERENCE_FRAME)
        # Allow 5 seconds per planning attempt
        arm.set_planning_time(5)

        rospy.sleep(1)

        result = None

        if req.move_id == NEUTRAL:
            # gripper.set_joint_value_target(GRIPPER_NEUTRAL_VAL)
            gripper.set_named_target('l_gripper_init')
            result = gripper.go()
            # arm.set_joint_value_target(ARM_NEUTRAL_VAL)
            arm.set_named_target('l_arm_init')
            result = arm.go()
        elif req.move_id == ARM_NEUTRAL:
            # arm.set_joint_value_target(ARM_NEUTRAL_VAL)
            arm.set_named_target('l_arm_init')
            result = arm.go()
        elif req.move_id == ARM_GRASPING:
            # arm.set_joint_value_target(ARM_GRASPING_VAL)
            arm.set_named_target('l_arm_grasping')
            result = arm.go()
        elif req.move_id == ARM_GRASPING2:
            # arm.set_joint_value_target(ARM_GRASPING2_VAL)
            arm.set_named_target('l_arm_grasping2')
            result = arm.go()
        elif req.move_id == ARM_GIVE:
            # arm.set_joint_value_target(ARM_GIVE_VAL)
            arm.set_named_target('l_arm_give')
            result = arm.go()
        elif req.move_id == ARM_GIVE_END:
            # arm.set_joint_value_target(ARM_GIVE_END_VAL)
            arm.set_named_target('l_arm_give_end')
            result = arm.go()
        elif req.move_id == ARM_SERVING:
            arm.set_named_target('l_arm_serving_pose')
            result = arm.go()
        elif req.move_id == GRIPPER_NEUTRAL:
            # gripper.set_joint_value_target(GRIPPER_NEUTRAL_VAL)
            gripper.set_named_target('l_gripper_init')
            result = gripper.go()
        elif req.move_id == GRIPPER_OPEN:
            # gripper.set_joint_value_target(GRIPPER_OPEN_VAL)
            gripper.set_named_target('l_gripper_open')
            result = gripper.go()
            arm.detach_object()
            gripper.detach_object()
            scene.remove_attached_object(GRIPPER_FRAME,str(req.object_id))
            scene.remove_world_object(str(req.object_id))
        elif req.move_id == GRIPPER_CLOSE:
            # gripper.set_joint_value_target(GRIPPER_CLOSE_VAL)
            gripper.set_named_target('l_gripper_close')
            result = gripper.go()
        else:
            rospy.logerr("An illigal move_id")
            result = False

        ret = rp_arm_moveResponse()
        ret.result = result
        return ret

    def shutdown(self):
        rospy.loginfo("Stopping the node")
        # Shut down MoveIt cleanly and exit the script
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        SubTaskArmMove()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("subtask_arm_move node terminated.")
