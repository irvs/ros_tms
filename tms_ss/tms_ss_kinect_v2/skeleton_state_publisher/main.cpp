#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>

#include <tms_ss_kinect_v2/SkeletonArray.h>

#include "../calc_joint_angles/for_model01.h"

class SkeletonStatePublisher : public robot_state_publisher::RobotStatePublisher
{
  public:
    tf::TransformBroadcaster broadcaster;

    SkeletonStatePublisher(const KDL::Tree& tree);
    ~SkeletonStatePublisher();

    void run();
    void send(ros::Time time);

  private:
    ros::NodeHandle nh;
    ros::Subscriber data_sub;
    ros::Publisher state_pub;
    sensor_msgs::JointState state_data;
    tf::StampedTransform transform_;

    std::map<std::string, double> joint_states;

    void callback(const tms_ss_kinect_v2::SkeletonArray::ConstPtr& msg);
};

//------------------------------------------------------------------------------
SkeletonStatePublisher::SkeletonStatePublisher(const KDL::Tree& tree) :
  robot_state_publisher::RobotStatePublisher(tree)
{
  //data_sub = nh.subscribe("integrated_skeleton_stream", 1,
  //    &SkeletonStatePublisher::callback, this);
  transform_ = tf::StampedTransform(
      tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0), tf::Vector3(0.0,0.0,0.0)),
      ros::Time::now(), "world_link", "Body");
  return;
}

//------------------------------------------------------------------------------
SkeletonStatePublisher::~SkeletonStatePublisher()
{
  return;
}

//------------------------------------------------------------------------------
void SkeletonStatePublisher::callback(const tms_ss_kinect_v2::SkeletonArray::ConstPtr& msg)
{
  //for (int i=0; i<1; i++)
  //{
  //  state_data.header.stamp = ros::Time::now();

  //  std::map<std::string, float> joint_angles;
  //  calcJointAnglesForModel01<float>(msg->data[i], joint_angles);

  //  state_data.name.clear();
  //  state_data.position.clear();
  //  for (int j=0; j<kJointDoF; j++)
  //  {
  //    state_data.name.push_back(kJointName[j]);
  //    state_data.position.push_back(0.0);
  //    state_data.velocity.push_back(0.0);
  //    state_data.effort.push_back(0.0);
  //    //state_data.position.push_back(joint_angles[kJointName[j]]);
  //  }
  //  ROS_INFO("Publish skeleton joint state.");
  //}
  return;
}

//------------------------------------------------------------------------------
void SkeletonStatePublisher::run()
{
  double yaw = 0.0;
  ros::Duration sleeper(0.1);
  while (nh.ok())
  {
    ros::Time time = ros::Time::now() + sleeper;
    for (int j=0; j<kJointDoF; j++)
    {
      joint_states[kJointName[j]] = 0.0;
    }
    this->send(time);
    /* v Testing v */
    tf::Quaternion q;
    q.setRPY(0,0,yaw);
    transform_.setData(tf::Transform(q, tf::Vector3(1.0,0.0,0.0)));
    yaw += 0.1;
    /* ^ Testing ^ */
    ros::spinOnce();
    sleeper.sleep();
  }
  return;
}

//------------------------------------------------------------------------------
void SkeletonStatePublisher::send(ros::Time time)
{
  transform_.stamp_ = time;
  broadcaster.sendTransform(transform_);
  this->publishTransforms(joint_states, time, "");
  return;
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ROS_INFO("Start program");
  ros::init(argc, argv, "skeleton_state_publisher");
  
  urdf::Model model;
  model.initParam("robot_description");
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return -1;
  }

  SkeletonStatePublisher obj(tree);

  obj.run();

  return 0;
}
