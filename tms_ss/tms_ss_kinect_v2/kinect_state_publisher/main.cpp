#include <sstream>

#include <Eigen/Eigen>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <robot_state_publisher/robot_state_publisher.h>

#include <tms_ss_kinect_v2/SkeletonStreamWrapper.h>

//------------------------------------------------------------------------------
class KinectStatePublisher : public robot_state_publisher::RobotStatePublisher
{
  public:
    tf::TransformBroadcaster broadcaster;

    KinectStatePublisher(const KDL::Tree& tree, int assigned_number=0);
    ~KinectStatePublisher();

    void run();
    void send(ros::Time time);

  private:
    ros::NodeHandle nh;
    ros::Subscriber data_sub;
    ros::Publisher state_pub;
    sensor_msgs::JointState state_data;
    tf::StampedTransform transform_;

    int assigned_number_;
    std::string tf_prefix_;

    Eigen::Vector3d pos;
    Eigen::Quaterniond rot;

    void callback(const tms_ss_kinect_v2::SkeletonStreamWrapper::ConstPtr& msg);
};

//------------------------------------------------------------------------------
KinectStatePublisher::KinectStatePublisher(const KDL::Tree& tree, int assigned_number) :
  robot_state_publisher::RobotStatePublisher(tree),
  assigned_number_(assigned_number),
  tf_prefix_("")
{
  if (assigned_number != 0)
  {
    std::stringstream ss;
    ss << "kinect" << assigned_number_;
    tf_prefix_ = ss.str();
  }

  std::stringstream subscribe_topic;
  subscribe_topic << "skeleton_stream_wrapper" << assigned_number;
  data_sub = nh.subscribe(subscribe_topic.str(), 1,
      &KinectStatePublisher::callback, this);
  transform_ = tf::StampedTransform(
      tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0), tf::Vector3(0.0,0.0,0.0)),
      ros::Time::now(), "world_link", tf::resolve(tf_prefix_,"base_link"));

  rot = Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0);
  pos = Eigen::Vector3d(0.0, 0.0, 0.0);

  return;
}

//------------------------------------------------------------------------------
KinectStatePublisher::~KinectStatePublisher()
{
  return;
}

//------------------------------------------------------------------------------
void KinectStatePublisher::callback(const tms_ss_kinect_v2::SkeletonStreamWrapper::ConstPtr& msg)
{
  if (msg->camera_number == assigned_number_)
  {
    ROS_INFO("kinect%d: Received posture.", assigned_number_);
    tms_ss_kinect_v2::CameraPosture camera_posture = msg->camera_posture;
    rot = Eigen::Quaterniond(
        camera_posture.rotation.w,
        camera_posture.rotation.x,
        camera_posture.rotation.y,
        camera_posture.rotation.z);
    pos = Eigen::Vector3d(
        camera_posture.translation.x,
        camera_posture.translation.y,
        camera_posture.translation.z);
  }
  else
  {
    ROS_ERROR("kinect%d: Received unexpected data.", assigned_number_);
  }

  return;
}

//------------------------------------------------------------------------------
void KinectStatePublisher::run()
{
  ros::Duration sleeper(10.0);
  while (nh.ok())
  {
    ros::Time time = ros::Time::now() + sleeper;

    ros::spinOnce();

    tf::Quaternion q(rot.x(), rot.y(), rot.z(), rot.w());
    transform_.setData(tf::Transform(q, tf::Vector3(pos[0],pos[1],pos[2])));

    this->send(time);
    sleeper.sleep();
  }
  return;
}

//------------------------------------------------------------------------------
void KinectStatePublisher::send(ros::Time time)
{
  std::map<std::string, double> joint_states;
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

  ros::init(argc, argv, "kinect_state_publisher");

  urdf::Model model;
  std::stringstream description_name_stream;
  description_name_stream << "kinect_description";

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

  KinectStatePublisher obj(tree, assigned_number);

  obj.run();

  return 0;
}

