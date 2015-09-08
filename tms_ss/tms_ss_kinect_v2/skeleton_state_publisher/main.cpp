#include <sstream>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <robot_state_publisher/robot_state_publisher.h>

#include <tms_ss_kinect_v2/SkeletonArray.h>

#include "../calc_joint_angles/for_model01.h"

//------------------------------------------------------------------------------
class SkeletonStatePublisher : public robot_state_publisher::RobotStatePublisher
{
  public:
    tf::TransformBroadcaster broadcaster;

    SkeletonStatePublisher(const KDL::Tree& tree, int user_id=0);
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
    tms_ss_kinect_v2::Skeleton skeleton;
    int user_id_;
    std::string tf_prefix_;
    bool bFind;

    void callback(const tms_ss_kinect_v2::SkeletonArray::ConstPtr& msg);
};

//------------------------------------------------------------------------------
SkeletonStatePublisher::SkeletonStatePublisher(const KDL::Tree& tree, int user_id) :
  robot_state_publisher::RobotStatePublisher(tree),
  user_id_(user_id),
  bFind(false),
  tf_prefix_("")
{
  if (user_id != 0)
  {
    std::stringstream ss;
    ss << "skeleton" << user_id_;
    tf_prefix_ = ss.str();
  }

  data_sub = nh.subscribe("integrated_skeleton_stream", 1,
      &SkeletonStatePublisher::callback, this);
  transform_ = tf::StampedTransform(
      tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0), tf::Vector3(0.0,0.0,0.0)),
      ros::Time::now(), "world_link", tf::resolve(tf_prefix_,"Body"));
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
  skeleton = msg->data[user_id_-1];
  if (skeleton.user_id == user_id_)
  {
    bFind = true;
  }

  return;
}

//------------------------------------------------------------------------------
void SkeletonStatePublisher::run()
{
  ros::Duration sleeper(0.1);
  while (nh.ok())
  {
    ros::Time time = ros::Time::now() + sleeper;

    for (int j=0; j<kJointDoF; j++)
    {
      joint_states[kJointName[j]] = 0.0;
    }
    /* v Testing v */

    ros::spinOnce();
    if (bFind)
    {
      Eigen::Vector3d pos;
      Eigen::Quaterniond rot;

      calcForModel01<double>(skeleton, pos, rot, joint_states);

      tf::Quaternion q(rot.x(), rot.y(), rot.z(), rot.w());
      transform_.setData(tf::Transform(q, tf::Vector3(pos[0],pos[1],pos[2])));
    }

    /* ^ Testing ^ */
    this->send(time);
    bFind = false;
    sleeper.sleep();
  }
  return;
}

//------------------------------------------------------------------------------
void SkeletonStatePublisher::send(ros::Time time)
{
  transform_.stamp_ = time;
  broadcaster.sendTransform(transform_);
  this->publishTransforms(joint_states, time, tf_prefix_);
  return;
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  int user_id = 0;
  if (argc > 1)
  {
    user_id = atoi(argv[1]);
  }

  ros::init(argc, argv, "skeleton_state_publisher");

  urdf::Model model;
  std::stringstream description_name_stream;
  description_name_stream << "skeleton_description";

  //if (user_id != 0) // if given user ID.
  //{
  //  description_name_stream << user_id;
  //}
  model.initParam(description_name_stream.str());

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return -1;
  }

  SkeletonStatePublisher obj(tree, user_id);

  obj.run();

  return 0;
}

