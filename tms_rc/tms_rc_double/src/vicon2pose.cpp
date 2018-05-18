#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tms_msg_ss/vicon_data.h>

ros::Publisher pose_pub;

void ViconCallback(const tms_msg_ss::vicon_data::ConstPtr& arg) {
  if ("2010#wheelchair_blue" != arg->subjectName) {
    // if("2007#wheelchair" != arg->subjectName){
    return;
  }
  if (arg->rotation.x == 0.0 && arg->rotation.y == 0.0 && arg->rotation.z == 0.0 &&
      arg->rotation.y == 0.0) {
    ROS_WARN("Vicon have some error on reading pose. Check marker position ");
    return;
  }
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.frame_id = arg->header.frame_id;
  if ("/world" == pose.header.frame_id) {
    pose.header.frame_id = "world";
  }
  pose.header.stamp = ros::Time::now();
  pose.pose.pose.position = arg->translation;
  pose.pose.pose.position.x *= 0.001;
  pose.pose.pose.position.y *= 0.001;
  pose.pose.pose.position.z = 0.0;
  pose.pose.pose.orientation = arg->rotation;
  pose.pose.covariance.at(0) = pow(0.1, 2);
  pose.pose.covariance.at(7) = pow(0.1, 2);
  pose.pose.covariance.at(35) = pow(5.0 * M_PI / 180.0, 2);
  pose_pub.publish(pose);
}

int main(int argc, char** argv) {
  ROS_INFO("vicon2pose");
  ros::init(argc, argv, "vicon2pose");
  ros::NodeHandle nh;

  ros::Subscriber pf_predict_sub =
      nh.subscribe<tms_msg_ss::vicon_data>("vicon_stream/output", 10, ViconCallback);
  pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("vicon_pose", 5);

  ros::spin();
}