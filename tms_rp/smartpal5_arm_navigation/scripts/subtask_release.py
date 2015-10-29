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
GRIPPER_OPEN = [-1.0]
GRIPPER_CLOSED = [-0.7]
GRIPPER_NEUTRAL = [0.0]
GRIPPER_JOINT_NAMES = ['l_gripper_thumb_joint']
GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'world_link'

INIT_ARM_VALUE = [0.0, 0.08,0.0,0.0,0.0,0.0,0.0]

class SubTaskRelease:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('subtask_release')
        rospy.on_shutdown(self.shutdown)

        self.release_srv = rospy.Service('subtask_release', rp_release, self.releaseSrvCallback)

    def releaseSrvCallback(self, req):
        rospy.loginfo("Received the service call!")
        rospy.loginfo(req)

        temp_dbdata = Tmsdb()
        target = Tmsdb()

        temp_dbdata.id = req.object_id

        rospy.wait_for_service('tms_db_reader')
        try:
            tms_db_reader = rospy.ServiceProxy('tms_db_reader', TmsdbGetData)
            res = tms_db_reader(temp_dbdata)
            target = res.tmsdb[0]
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            self.shutdown()

        print(target.name)

        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped)
        # Create a dictionary to hold object colors
        self.colors = dict()

        # Initialize the move group for the right arm
        arm = MoveGroupCommander(GROUP_NAME_ARM)
        # Initialize the move group for the right gripper
        gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()

        # Allow some leeway in position (meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.05)
        arm.set_goal_orientation_tolerance(0.1)

        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)

        # Set the right arm reference frame
        arm.set_pose_reference_frame(REFERENCE_FRAME)
        # Allow 5 seconds per planning attempt
        arm.set_planning_time(5)
        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 10
        # Set a limit on the number of place attempts
        max_place_attempts = 10
        # Give the scene a chance to catch up
        rospy.sleep(2)

        target_id = str(req.object_id)

        rospy.sleep(1)

        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = req.x
        target_pose.pose.position.y = req.y
        target_pose.pose.position.z = req.z
        # q = quaternion_from_euler(target.rr, target.rp, target.ry)
        q = quaternion_from_euler(req.roll, req.pitch, req.yaw)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        print(target_pose.pose.position.x)
        print(target_pose.pose.position.y)
        print(target_pose.pose.position.z)

        # Initialize the grasp pose to the target pose
        place_pose = target_pose

        # Shift the grasp pose by half the width of the target to center it
        # place_pose.pose.position.x -= target_size[0] / 2.0 + 0.01
        # place_pose.pose.position.y -= target_size[1] / 2.0

        # Generate a list of grasps
        places = self.make_places(place_pose)

        # Track success/failure and number of attempts for pick operation
        result = None
        n_attempts = 0
        print('test4')
        # Repeat until we succeed or run out of attempts
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
            n_attempts += 1
            rospy.loginfo("Place attempt: " +  str(n_attempts))
            for place in places:
                result = arm.place(target_id, place)
                print(result)
                if result == MoveItErrorCodes.SUCCESS:
                    break
            rospy.sleep(0.2)

        gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        gripper.go()
        arm.set_joint_value_target(INIT_ARM_VALUE)
        arm.go()

        ret = rp_releaseResponse()
        # If the pick was successful, attempt the place operation
        if result == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Success the place operation")
            ret.result = True
        else:
            rospy.loginfo("Place operation failed after " + str(n_attempts) + " attempts.")
            ret.result = False

        return ret

    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, joint_positions):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()

        # Set the joint names to the gripper joint names
        t.joint_names = GRIPPER_JOINT_NAMES

        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()

        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions

        # Set the gripper effort
        tp.effort = GRIPPER_EFFORT

        tp.time_from_start = rospy.Duration(1.0)

        # Append the goal point to the trajectory points
        t.points.append(tp)

        # Return the joint trajectory
        return t

    # Generate a gripper translation in the direction given by vector
    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()

        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]

        # The vector is relative to the gripper frame
        g.direction.header.frame_id = GRIPPER_FRAME

        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired

        return g

    # Generate a list of possible grasps
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
        # Initialize the grasp object
        g = Grasp()

        # Set the pre-grasp and grasp postures appropriately
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)

        # Set the approach and retreat parameters as desired
        g.pre_grasp_approach = self.make_gripper_translation(0.1, 0.2, [0.0, 1.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.2, [0.0, 0.0, 1.0])

        # Set the first grasp pose to the input pose
        g.grasp_pose = initial_pose_stamped

        # Pitch angles to try
        pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.4, -0.4]

        # Yaw angles to try
        yaw_vals = [0]

        # A list to hold the grasps
        grasps = []

        # Generate a grasp for each pitch and yaw angle
        for y in yaw_vals:
            for p in pitch_vals:
                # Create a quaternion from the Euler angles
                q = quaternion_from_euler(0, p, y)

                # # Set the grasp pose orientation accordingly
                g.grasp_pose.pose.orientation.x = q[0]
                g.grasp_pose.pose.orientation.y = q[1]
                g.grasp_pose.pose.orientation.z = q[2]
                g.grasp_pose.pose.orientation.w = q[3]

                # Set and id for this grasp (simply needs to be unique)
                g.id = str(len(grasps))

                # Set the allowed touch objects to the input list
                g.allowed_touch_objects = allowed_touch_objects

                # Don't restrict contact force
                g.max_contact_force = 0

                # Degrade grasp quality for increasing pitch angles
                g.grasp_quality = 1.0 - abs(p)

                # Append the grasp to the list
                grasps.append(deepcopy(g))

        # Return the list
        return grasps

    # Generate a list of possible place poses
    def make_places(self, init_pose):
        # Initialize the place location as a PoseStamped message
        place = PoseStamped()

        # Start with the input place pose
        place = init_pose

        # A list of x shifts (meters) to try
        x_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]

        # A list of y shifts (meters) to try
        y_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]

        pitch_vals = [0]

        # A list of yaw angles to try
        yaw_vals = [0]

        # A list to hold the places
        places = []

        # Generate a place pose for each angle and translation
        for y in yaw_vals:
            for p in pitch_vals:
                for y in y_vals:
                    for x in x_vals:
                        place.pose.position.x = init_pose.pose.position.x + x
                        place.pose.position.y = init_pose.pose.position.y + y

                        # Create a quaternion from the Euler angles
                        # q = quaternion_from_euler(0, p, y)

                        # # Set the place pose orientation accordingly
                        # place.pose.orientation.x = q[0]
                        # place.pose.orientation.y = q[1]
                        # place.pose.orientation.z = q[2]
                        # place.pose.orientation.w = q[3]

                        q = quaternion_from_euler(0, p, y)
                        place.pose.orientation.x = q[0]
                        place.pose.orientation.y = q[1]
                        place.pose.orientation.z = q[2]
                        place.pose.orientation.w = q[3]

                        # Append this place pose to the list
                        places.append(deepcopy(place))

        # Return the list
        return places

    def shutdown(self):
        rospy.loginfo("Stopping the node")
        # Shut down MoveIt cleanly and exit the script
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        SubTaskRelease()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("subtask_release node terminated.")
