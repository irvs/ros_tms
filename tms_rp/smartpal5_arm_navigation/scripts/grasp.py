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

GROUP_NAME_ARM = 'l_arm'
GROUP_NAME_GRIPPER = 'l_gripper'

GRIPPER_FRAME = 'l_end_effector_link'

GRIPPER_OPEN = [-1.0]
GRIPPER_CLOSED = [-0.3]
GRIPPER_NEUTRAL = [0.0]

GRIPPER_JOINT_NAMES = ['l_gripper_thumb_joint']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'world_link'

class Grasp:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('grasp')

        scene = PlanningSceneInterface()
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped)
        self.colors = dict()

        arm = MoveGroupCommander(GROUP_NAME_ARM)
        gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        end_effector_link = arm.get_end_effector_link()

        arm.set_goal_position_tolerance(0.05)
        arm.set_goal_orientation_tolerance(0.1)
        arm.allow_replanning(True)
        arm.set_pose_reference_frame(REFERENCE_FRAME)
        arm.set_planning_time(5)
        max_pick_attempts = 5
        max_place_attempts = 5

        rospy.sleep(2)

        table_id = 'table'
        target_id = 'target'
        scene.remove_world_object(table_id)
        scene.remove_world_object(target_id)
        scene.remove_attached_object(GRIPPER_FRAME, target_id)

        rospy.sleep(1)

        arm.set_named_target('l_arm_init')
        arm.go()

        gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        gripper.go()

        rospy.sleep(1)

        table_ground = 0.65
        table_size = [0.2, 0.7, 0.01]
        target_size = [0.03, 0.03, 0.12]

        table_pose = PoseStamped()
        table_pose.header.frame_id = REFERENCE_FRAME
        table_pose.pose.position.x = 0.35
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0

        scene.add_box(table_id, table_pose, table_size)

        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = 0.35
        target_pose.pose.position.y = 0.2
        target_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
        q = quaternion_from_euler(0, 0, -1.57079633)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        scene.add_box(target_id, target_pose, target_size)

        self.setColor(table_id, 0.8, 0, 0, 1.0)
        self.setColor(target_id, 0.9, 0.9, 0, 1.0)
        self.sendColors()

        arm.set_support_surface_name(table_id)

        rospy.sleep(1)

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def setColor(self, name, r, g, b, a = 0.9):
        color = ObjectColor()
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        self.colors[name] = color

    def sendColors(self):
        p = PlanningScene()
        p.is_diff = True
        for color in self.colors.values():
            p.object_colors.append(color)
        self.scene_pub.publish(p)

if __name__ == "__main__":
    Grasp()
