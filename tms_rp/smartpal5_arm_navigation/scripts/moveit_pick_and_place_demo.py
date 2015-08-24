#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from control_msgs.msg import GripperCommand
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler
from copy import deepcopy

GROUP_NAME_ARM = 'arm_left'
GROUP_NAME_GRIPPER = 'gripper_left'

GRIPPER_FRAME = 'leftGripper_link'

GRIPPER_OPEN = [-0.8]
GRIPPER_CLOSED = [-0.1]
GRIPPER_NEUTRAL = [0.0]

GRIPPER_JOINT_NAMES = ['leftGripper__leftGripper_thumb_joint']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'start_position'

class MoveItPickAndPlaceDemo:
    def __init__(self):
        # Initialize the move_group API and node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_pick_and_place_demo', anonymous=True)

        # Use the groups of SmartPal5
        arm_left = moveit_commander.MoveGroupCommander('arm_left')
        gripper_left = moveit_commander.MoveGroupCommander('gripper_left')
        end_effector_link = arm_left.get_end_effector_link() # leftGripper_link
        reference_frame = 'start_position'
        scene = PlanningSceneInterface()

        # Set publisher
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=10)

        self.colors = dict()

        # Set a goal joint tolerance
        arm_left.set_goal_joint_tolerance(0.05)
        arm_left.set_goal_orientation_tolerance(0.1)

        # Set the option related IK solution
        arm_left.allow_replanning(True)
        arm_left.set_planning_time(5)
        arm_left.set_pose_reference_frame(reference_frame)

        # Set the scene
        name_of_table = 'table'
        name_of_target_object = 'target'

        scene.remove_world_object(name_of_table)
        scene.remove_world_object(name_of_target_object)
        scene.remove_attached_object('leftGripper_link', name_of_target_object)

        rospy.sleep(1)

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

        # Set the environment with table and target object
        height_of_table = 0.65
        size_of_table = [0.2, 0.7, 0.01]
        size_of_target_object = [0.03, 0.03, 0.12]

        pose_of_table = PoseStamped()
        pose_of_table.header.frame_id = 'start_position'
        pose_of_table.pose.position.x = 0.35
        pose_of_table.pose.position.y = 0.0
        pose_of_table.pose.position.z = height_of_table + size_of_table[2] / 2.0
        pose_of_table.pose.orientation.w = 1.0

        scene.add_box(name_of_table, pose_of_table, size_of_table)  
        
        # Set the target pose in between the boxes and on the table
        pose_of_target_object = PoseStamped()
        pose_of_target_object.header.frame_id = 'start_position'
        pose_of_target_object.pose.position.x = 0.35
        pose_of_target_object.pose.position.y = 0.2
        pose_of_target_object.pose.position.z = height_of_table + size_of_table[2] + size_of_target_object[2] / 2.0
        pose_of_target_object.pose.orientation.w = 1.0
        
        scene.add_box(name_of_target_object, pose_of_target_object, size_of_target_object)

        self.setColor(name_of_table, 0.8, 0, 0, 1.0)
        self.setColor(name_of_target_object, 0.8, 0.4, 0, 1.0)
        self.sendColors()

        # Use the pick and place features
        arm_left.set_support_surface_name(name_of_table)
        
        pose_of_place = PoseStamped()
        pose_of_place.header.frame_id = 'start_position'
        pose_of_place.pose.position.x = 0.18
        pose_of_place.pose.position.y = -0.18
        pose_of_place.pose.position.z = height_of_table + size_of_table[2] + size_of_target_object[2] / 2.0
        pose_of_place.pose.orientation.w = 1.0

        pose_of_grasp = pose_of_target_object

        pose_of_grasp.pose.position.y -= size_of_target_object[1] / 2.0

        grasps = self.make_grasps(pose_of_grasp, [name_of_target_object])

        for grasp in grasps:
            self.gripper_pose_pub.publish(grasp.grasp_pose)
            rospy.sleep(0.2)

        result = None
        n_attempts = 0

        while result != MoveItErrorCodes.SUCCESS and n_attempts < 20:
            n_attempts += 1
            rospy.loginfo("Pick attempt: " +  str(n_attempts))
            result = arm_left.pick(name_of_target_object, grasps)
            rospy.sleep(0.2)

        if result == MoveItErrorCodes.SUCCESS:
            result = None
            n_attempts = 0
            
            # Generate valid place poses
            places = self.make_places(pose_of_place)
            
            # Repeat until we succeed or run out of attempts
            while result != MoveItErrorCodes.SUCCESS and n_attempts < 5:
                n_attempts += 1
                rospy.loginfo("Place attempt: " +  str(n_attempts))
                for place in places:
                    result = arm_left.place(name_of_target_object, place)
                    if result == MoveItErrorCodes.SUCCESS:
                        break
                rospy.sleep(0.2)
                
            if result != MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Place operation failed after " + str(n_attempts) + " attempts.")
        else:
            rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")
                 
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
        g.pre_grasp_approach = self.make_gripper_translation(0.01, 0.1, [1.0, 0.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, [0.0, -1.0, 1.0])
        
        # Set the first grasp pose to the input pose
        g.grasp_pose = initial_pose_stamped
    
        # Pitch angles to try
        pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3]
        
        # Yaw angles to try
        yaw_vals = [0]

        # A list to hold the grasps
        grasps = []

        # Generate a grasp for each pitch and yaw angle
        for y in yaw_vals:
            for p in pitch_vals:
                # Create a quaternion from the Euler angles
                q = quaternion_from_euler(0, p, y)
                
                # Set the grasp pose orientation accordingly
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
                        q = quaternion_from_euler(0, p, y)
                        
                        # Set the place pose orientation accordingly
                        place.pose.orientation.x = q[0]
                        place.pose.orientation.y = q[1]
                        place.pose.orientation.z = q[2]
                        place.pose.orientation.w = q[3]
                        
                        # Append this place pose to the list
                        places.append(deepcopy(place))
        
        # Return the list
        return places
    
    # Set the color of an object
    def setColor(self, name, r, g, b, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()
        
        # Set the id to the name given as an argument
        color.id = name
        
        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff        
        p.is_diff = True
        
        # Append the colors from the global color dictionary 
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # Publish the scene diff
        self.scene_pub.publish(p)

if __name__ == "__main__":
    try:
        MoveItPickAndPlaceDemo()
    except rospy.ROSInterruptException:
        pass
