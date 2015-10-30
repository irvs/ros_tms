#include <sstream>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <robot_state_publisher/robot_state_publisher.h>

#include <tms_msg_ss/SkeletonArray.h>

#include "../calc_joint_angles/for_model01.h"

#define GMT 9 * 60 * 60  // GMT: Tokyo +9

//------------------------------------------------------------------------------
class SkeletonStatePublisher : public robot_state_publisher::RobotStatePublisher
{
  public:
    tf::TransformBroadcaster broadcaster;

    SkeletonStatePublisher(const KDL::Tree& tree, int assigned_number=0);
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
    tms_msg_ss::Skeleton skeleton;
    int assigned_number_;
    std::string tf_prefix_;
    bool bFind;

    void callback(const tms_msg_ss::SkeletonArray::ConstPtr& msg);
};

//------------------------------------------------------------------------------
SkeletonStatePublisher::SkeletonStatePublisher(const KDL::Tree& tree, int assigned_number) :
  robot_state_publisher::RobotStatePublisher(tree),
  assigned_number_(assigned_number),
  bFind(false),
  tf_prefix_("")
{
  if (assigned_number != 0)
  {
    std::stringstream ss;
    ss << "skeleton" << assigned_number_;
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
void SkeletonStatePublisher::callback(const tms_msg_ss::SkeletonArray::ConstPtr& msg)
{
  skeleton = msg->data[assigned_number_-1];
  if (skeleton.user_id+1 > 0)
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
    ros::Time time = ros::Time::now() + ros::Duration(GMT) + sleeper;
    /* v Testing v */

    ros::spinOnce();

    Eigen::Vector3d pos;
    Eigen::Quaterniond rot;
    if (bFind)
    {
      calcForModel01<double>(skeleton, pos, rot, joint_states);
      tf::Quaternion q(rot.x(), rot.y(), rot.z(), rot.w());
      transform_.setData(tf::Transform(q, tf::Vector3(pos[0],pos[1],pos[2])));
    }
    else
    {
      for (int j=0; j<kJointDoF; j++)
      {
        joint_states[kJointName[j]] = 0.0;
      }
      tf::Quaternion q(0.0, 0.0, 0.0, 1.0);
      transform_.setData(tf::Transform(q, tf::Vector3(0.0,0.0,0.0)));
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
  int assigned_number = 0;
  if (argc > 1)
  {
    assigned_number = atoi(argv[1]);
  }

  ros::init(argc, argv, "skeleton_state_publisher");

  urdf::Model model;
  std::stringstream description_name_stream;
  description_name_stream << "skeleton_description";

  if (assigned_number != 0) // if given user ID.
  {
    description_name_stream << assigned_number;
  }
  model.initParam(description_name_stream.str());

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return -1;
  }

  SkeletonStatePublisher obj(tree, assigned_number);

  obj.run();

  return 0;
}

