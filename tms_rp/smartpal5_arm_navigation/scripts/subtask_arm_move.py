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
ARM_GIVE = 3
ARM_GIVE_END = 4

GRIPPER_NEUTRAL = 100
GRIPPER_OPEN = 101
GRIPPER_CLOSE = 102

ARM_NEUTRAL_VAL = [0.0, -0.08,0.0,0.0,0.0,0.0,0.0]
ARM_GRASPING_VAL = [0.0, -0.2,0.0,0.0,0.0,0.0,0.0]
ARM_GIVE_VAL = [0.65,-0.08,0.0,0.93,0.0,0.0,0.0]
ARM_GIVE_END_VAL = [0.65,-0.5,0.0,0.93,0.0,0.0,0.0]

GRIPPER_NEUTRAL_VAL = [0.0]
GRIPPER_OPEN_VAL = [-1.0]
GRIPPER_CLOSE_VAL = [-0.7]

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

        rospy.sleep(1)

        result = None

        if req.move_id == NEUTRAL:
            gripper.set_joint_value_target(GRIPPER_NEUTRAL_VAL)
            result = gripper.go()
            arm.set_joint_value_target(ARM_NEUTRAL_VAL)
            result = arm.go()
        elif req.move_id == ARM_NEUTRAL:
            arm.set_joint_value_target(ARM_NEUTRAL_VAL)
            result = arm.go()
        elif req.move_id == ARM_GRASPING:
            arm.set_joint_value_target(ARM_GRASPING_VAL)
            result = arm.go()
        elif req.move_id == ARM_GIVE:
            arm.set_joint_value_target(ARM_GIVE_VAL)
            result = arm.go()
        elif req.move_id == ARM_GIVE_END:
            arm.set_joint_value_target(ARM_GIVE_END_VAL)
            result = arm.go()
        elif req.move_id == GRIPPER_NEUTRAL:
            gripper.set_joint_value_target(GRIPPER_NEUTRAL_VAL)
            result = gripper.go()
        elif req.move_id == GRIPPER_OPEN:
            gripper.set_joint_value_target(GRIPPER_OPEN_VAL)
            result = gripper.go()
            arm.detach_object()
            gripper.detach_object()
            scene.remove_attached_object(GRIPPER_FRAME,str(req.object_id))
            scene.remove_world_object(str(req.object_id))
        elif req.move_id == GRIPPER_CLOSE:
            gripper.set_joint_value_target(GRIPPER_CLOSE_VAL)
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
