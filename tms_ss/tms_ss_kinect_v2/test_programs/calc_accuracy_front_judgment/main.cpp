#include <iostream>
#include <limits>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tms_msg_ss/SkeletonArray.h>
#include <tms_msg_ss/vicon_data.h>

#include <for_model02.h>

#define EMPTY_SKELETON_ID -1
#define VICON_OBJECT_NAME "moverio"

int num_of_trials;
int accurate_num;
ros::Subscriber vicon_sub;
ros::Subscriber skeleton_sub;
ros::Publisher vicon_pub;
ros::Publisher skeleton_pub;

visualization_msgs::Marker vicon_obj_arrow;
visualization_msgs::Marker skeleton_arrow;

//----------------------------------------------------------------------------
inline void calcAccuracy()
{
	if (num_of_trials != std::numeric_limits<int>::max()) {num_of_trials++;}
	else { ROS_WARN("Trial counter reached limit."); }

	Eigen::Quaterniond q1(
			vicon_obj_arrow.pose.orientation.w,
			vicon_obj_arrow.pose.orientation.x,
			vicon_obj_arrow.pose.orientation.y,
			vicon_obj_arrow.pose.orientation.z);
	Eigen::Quaterniond q2(
			skeleton_arrow.pose.orientation.w,
			skeleton_arrow.pose.orientation.x,
			skeleton_arrow.pose.orientation.y,
			skeleton_arrow.pose.orientation.z);
	Eigen::Vector3d v1(q1*Eigen::Vector3d::UnitX());
	Eigen::Vector3d v2(q2*Eigen::Vector3d::UnitX());

	if (v1.dot(v2) >= 0)
	{
		if (num_of_trials != std::numeric_limits<int>::max()) {accurate_num++;}
		else { ROS_WARN("Trial counter reached limit."); }
	}

	ROS_INFO("Trial: %05d, Correct: %05d, Accurate Ratio: %3.1f%%",
			num_of_trials, accurate_num, (double)accurate_num/(double)num_of_trials*100.0);

	return;
}

//----------------------------------------------------------------------------
inline void draw()
{
	calcAccuracy();
	vicon_pub.publish( vicon_obj_arrow );
	skeleton_pub.publish( skeleton_arrow );
	return;
}

//-----------------------------------------------------------------------------
void vicon_callback(const tms_msg_ss::vicon_data::ConstPtr& msg)
{
	if (msg->subjectName == VICON_OBJECT_NAME)
	{
		vicon_obj_arrow.header.frame_id = "world_link";
		vicon_obj_arrow.header.stamp = ros::Time();
		vicon_obj_arrow.ns = "";
		vicon_obj_arrow.id = 1;
		vicon_obj_arrow.type = visualization_msgs::Marker::ARROW;
		vicon_obj_arrow.action = visualization_msgs::Marker::MODIFY;
		vicon_obj_arrow.pose.position.x = msg->translation.x / 1000;
		vicon_obj_arrow.pose.position.y = msg->translation.y / 1000;
		vicon_obj_arrow.pose.position.z = msg->translation.z / 1000;
		vicon_obj_arrow.pose.orientation = msg->rotation;
		vicon_obj_arrow.scale.x = 1;
		vicon_obj_arrow.scale.y = 0.1;
		vicon_obj_arrow.scale.z = 0.1;
		vicon_obj_arrow.color.a = 1.0; // Don't forget to set the alpha!
		vicon_obj_arrow.color.r = 0.0;
		vicon_obj_arrow.color.g = 0.0;
		vicon_obj_arrow.color.b = 1.0;
	}
	return;
}

//-----------------------------------------------------------------------------
void skeleton_callback(const tms_msg_ss::SkeletonArray::ConstPtr& msg)
{
	int index;
	for (index=0;
			index < msg->data.size() && msg->data[index].user_id == EMPTY_SKELETON_ID;
	 	 	index++) { }

	if (index != msg->data.size())
	{
		std::vector<Eigen::Vector3d> joint_array;
		Eigen::Vector3d position;
		Eigen::Quaterniond orientation;
		geometry_msgs::Pose skeleton_pose;

		convertStructureToVector(msg->data[index], joint_array);
		makePositionVector<double>(joint_array, position);
		makeOrientationQuaternion<double>(joint_array, orientation);
		orientation = orientation * Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitZ());
		skeleton_pose.position.x = position[0];
		skeleton_pose.position.y = position[1];
		skeleton_pose.position.z = position[2];
		skeleton_pose.orientation.x = orientation.x();
		skeleton_pose.orientation.y = orientation.y();
		skeleton_pose.orientation.z = orientation.z();
		skeleton_pose.orientation.w = orientation.w();

		skeleton_arrow.header.frame_id = "world_link";
		skeleton_arrow.header.stamp = ros::Time();
		skeleton_arrow.ns = "";
		skeleton_arrow.id = 0;
		skeleton_arrow.type = visualization_msgs::Marker::ARROW;
		skeleton_arrow.action = visualization_msgs::Marker::MODIFY;
		skeleton_arrow.pose = skeleton_pose;
		skeleton_arrow.scale.x = 1;
		skeleton_arrow.scale.y = 0.1;
		skeleton_arrow.scale.z = 0.1;
		skeleton_arrow.color.a = 1.0; // Don't forget to set the alpha!
		skeleton_arrow.color.r = 0.0;
		skeleton_arrow.color.g = 1.0;
		skeleton_arrow.color.b = 0.0;

		draw();
	}

	return;
}

//----------------------------------------------------------------------------
int main (int argc, char **argv)
{
	ros::init(argc, argv, "skeleton_face_counter");

	ros::NodeHandle nh;

	// Initialize
	num_of_trials = 0;
	accurate_num = 0;
	vicon_sub = nh.subscribe("vicon_stream/output", 1, vicon_callback);
	skeleton_sub = nh.subscribe("integrated_skeleton_stream", 1, skeleton_callback);
  vicon_pub = nh.advertise<visualization_msgs::Marker>("skeleton_orientation_ground_truth", 1);
	skeleton_pub = nh.advertise<visualization_msgs::Marker>("skeleton_orientation", 1);

	// Loop
	ros::spin();

	return 0;
}
