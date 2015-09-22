#include <iostream>
#include <vector>
#include <queue>

#include <string>
#include <sstream>
#include <iomanip>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <tms_ss_kinect_v2/SkeletonArray.h>
#include <tms_ss_kinect_v2/SkeletonStreamWrapper.h>

#include "../calc_joint_angles/for_model01.h"

#include <unistd.h>

#define MAX_USERS 12

//-----------------------------------------------------------------------------
template <class T>
std::string to_str(const T& t)
{
  std::stringstream ss;
  ss << t;
  return ss.str();
}

//-----------------------------------------------------------------------------
inline tms_ss_kinect_v2::Skeleton initialize_skeleton()
{
  tms_ss_kinect_v2::Skeleton ret;
  ret.user_id = -1;
  ret.position.resize(25);
  ret.orientation.resize(25);
  ret.confidence.resize(25);
  for(int i=0; i<25; i++)
  {
    ret.position[i].x = 0.0;
    ret.position[i].y = 0.0;
    ret.position[i].z = 0.0;
    ret.orientation[i].w = 1.0;
    ret.orientation[i].x = 0.0;
    ret.orientation[i].y = 0.0;
    ret.orientation[i].z = 0.0;
    ret.confidence[i] = 0;
  }
  return ret;
}

//-----------------------------------------------------------------------------
class SkeletonIntegrator
{
  public:
    SkeletonIntegrator(const std::vector<int>& camera);
    ~SkeletonIntegrator();

    void callback(const tms_ss_kinect_v2::SkeletonStreamWrapper::ConstPtr& msg);
    void run();
  private:
    ros::NodeHandle nh;
    std::vector<int> array;
    tms_ss_kinect_v2::SkeletonArray skeletons;
    int new_user_;
    int tracked_skeleton_num_;
    int tracking_validity[MAX_USERS];
    std::vector<std::map<int, bool> > bFaceStateTable;  // [user_index, [camera_id, is_front?]]

    void listenSkeletonStream();

    // For Debug
    void showStatus();
};

//-----------------------------------------------------------------------------
SkeletonIntegrator::SkeletonIntegrator(const std::vector<int> &camera) :
  array(camera),
  new_user_(0),
  tracked_skeleton_num_(0)
{
  skeletons.data.resize(MAX_USERS);
  bFaceStateTable.resize(MAX_USERS);
  for (int i = 0; i < MAX_USERS; i++)
  {
    skeletons.data[i] = initialize_skeleton();
    tracking_validity[i] = 0;
    for (std::vector<int>::iterator it = array.begin(); it != array.end(); it++)
    {
      bFaceStateTable[i][*it] = false;
    }
  }
  return;
}

//-----------------------------------------------------------------------------
SkeletonIntegrator::~SkeletonIntegrator()
{
  return;
}

//-----------------------------------------------------------------------------
void SkeletonIntegrator::callback(const tms_ss_kinect_v2::SkeletonStreamWrapper::ConstPtr& msg)
{
  tms_ss_kinect_v2::Skeleton skeleton = msg->skeleton;
  tms_ss_kinect_v2::CameraPosture camera_posture = msg->camera_posture;

  // Transform to world coordinate
  Eigen::Vector3f translation(
      camera_posture.translation.x,
      camera_posture.translation.y,
      camera_posture.translation.z);
  Eigen::Quaternionf rotation(
      camera_posture.rotation.w,
      camera_posture.rotation.x,
      camera_posture.rotation.y,
      camera_posture.rotation.z);

  Eigen::Matrix3f rot_mat = rotation.matrix();
  ROS_INFO("Camera %d status:\n  \
      Position in world:\n    \
      (%3.4f, %3.4f, %3.4f)\n  \
      Rotation in world:\n    \
      x-axis (%3.4f, %3.4f, %3.4f)\n    \
      y-axis (%3.4f, %3.4f, %3.4f)\n    \
      z-axis (%3.4f, %3.4f, %3.4f)\n\
      \n",
        msg->camera_number,
        translation[0], translation[1], translation[2],
        rot_mat(0, 0), rot_mat(1, 0), rot_mat(2, 0),
        rot_mat(0, 1), rot_mat(1, 1), rot_mat(2, 1),
        rot_mat(0, 2), rot_mat(1, 2), rot_mat(2, 2));


  tms_ss_kinect_v2::Skeleton integrated_skeleton;
  integrated_skeleton.user_id = 0;
  integrated_skeleton.position.resize(25);
  integrated_skeleton.orientation.resize(25);
  integrated_skeleton.confidence.resize(25);
  for(int i=0; i<25; i++)
  {
    Eigen::Vector3f pos(
        skeleton.position[i].x,
        skeleton.position[i].y,
        skeleton.position[i].z);
    Eigen::Quaternionf ori(
        skeleton.orientation[i].w,
        skeleton.orientation[i].x,
        skeleton.orientation[i].y,
        skeleton.orientation[i].z);
    pos = rotation.matrix() * pos + translation;
    ori = rotation * ori;
    integrated_skeleton.position[i].x = pos[0];
    integrated_skeleton.position[i].y = pos[1];
    integrated_skeleton.position[i].z = pos[2];
    integrated_skeleton.orientation[i].w = ori.w();
    integrated_skeleton.orientation[i].x = ori.x();
    integrated_skeleton.orientation[i].y = ori.y();
    integrated_skeleton.orientation[i].z = ori.z();
    integrated_skeleton.confidence[i] = skeleton.confidence[i];
  }

  // identify received skeleton
  int index, user_id;
  bool already_detect = false;
  for (int i = 0; i < skeletons.data.size(); i++)
  {
    Eigen::Vector3f v1(
        integrated_skeleton.position[SpineMid].x,
        integrated_skeleton.position[SpineMid].y,
        integrated_skeleton.position[SpineMid].z);
    Eigen::Vector3f v2(
        skeletons.data[i].position[SpineMid].x,
        skeletons.data[i].position[SpineMid].y,
        skeletons.data[i].position[SpineMid].z);
    if ((v1-v2).norm() < 0.3)
    {
      index = i;
      user_id = skeletons.data[i].user_id;
      already_detect = true;
      break;
    }
  }
  if (!already_detect)
  {
    index = new_user_;
    user_id = tracked_skeleton_num_;
    for (int i=0; i < MAX_USERS && skeletons.data[(new_user_+i)%MAX_USERS].user_id >= 0; i++)
    {
      new_user_ = (new_user_ + i+1) % MAX_USERS;
    }
    tracked_skeleton_num_++;
  }

  // Update skeleton state
  bool &table_ref = bFaceStateTable[index][msg->camera_number];
  integrated_skeleton.user_id = user_id;
  if (msg->face_state == 2)
  {
    // Log face state and storage as valid skeleton
    table_ref = true;
  }
  else
  {
    if (table_ref)
    {
      Eigen::Quaternionf skeleton_rotation(
          integrated_skeleton.orientation[SpineBase].w,
          integrated_skeleton.orientation[SpineBase].x,
          integrated_skeleton.orientation[SpineBase].y,
          integrated_skeleton.orientation[SpineBase].z);
      Eigen::Vector3f skeleton_direction_cam;
      Eigen::Vector3f camera_direction_cam;
      // Calculate skeleton direction (front is y)
      Eigen::Vector3f tmp_x(
          skeleton.position[HipRight].x - skeleton.position[HipLeft].x,
          skeleton.position[HipRight].y - skeleton.position[HipLeft].y,
          skeleton.position[HipRight].z - skeleton.position[HipLeft].z);
      Eigen::Vector3f tmp_z(
          skeleton.position[SpineMid].x - skeleton.position[SpineBase].x,
          skeleton.position[SpineMid].y - skeleton.position[SpineBase].y,
          skeleton.position[SpineMid].z - skeleton.position[SpineBase].z);
      skeleton_direction_cam = (tmp_z.cross(tmp_x)).normalized();
      camera_direction_cam = Eigen::Vector3f::UnitZ();
      // Check that skeleton is facing the camera
      if (skeleton_direction_cam.dot(camera_direction_cam) < cos(150*M_PI/180))
      {
        // storage as valid skeleton
      }
      else
      {
        table_ref = false;
        // Allow only move
        const tms_ss_kinect_v2::Skeleton& last_state = skeletons.data[index];
        Eigen::Vector3d translation(
            integrated_skeleton.position[SpineMid].x - last_state.position[SpineMid].x,
            integrated_skeleton.position[SpineMid].y - last_state.position[SpineMid].y,
            integrated_skeleton.position[SpineMid].z - last_state.position[SpineMid].z);
        for (int i = 0; i < integrated_skeleton.position.size(); i++)
        {
          integrated_skeleton.position[i].x = last_state.position[i].x + translation[0];
          integrated_skeleton.position[i].y = last_state.position[i].y + translation[1];
          integrated_skeleton.position[i].z = last_state.position[i].z + translation[2];
        }
      }
    }
    else
    {
      // Allow only move
      const tms_ss_kinect_v2::Skeleton& last_state = skeletons.data[index];
      Eigen::Vector3d translation(
          integrated_skeleton.position[SpineMid].x - last_state.position[SpineMid].x,
          integrated_skeleton.position[SpineMid].y - last_state.position[SpineMid].y,
          integrated_skeleton.position[SpineMid].z - last_state.position[SpineMid].z);
      for (int i = 0; i < integrated_skeleton.position.size(); i++)
      {
        integrated_skeleton.position[i].x = last_state.position[i].x + translation[0];
        integrated_skeleton.position[i].y = last_state.position[i].y + translation[1];
        integrated_skeleton.position[i].z = last_state.position[i].z + translation[2];
      }
    }
  }
  skeletons.data[index] = integrated_skeleton;
  tracking_validity[index] = 10;  // reflesh validity

  return;
}

//-----------------------------------------------------------------------------
void SkeletonIntegrator::run()
{
  ros::Rate loop_late(10);

  static boost::thread thread_listener(
      boost::bind(&SkeletonIntegrator::listenSkeletonStream, this));

  ros::Publisher pub =
    nh.advertise<tms_ss_kinect_v2::SkeletonArray>(
        "integrated_skeleton_stream", 1);

  while (ros::ok())
  {
    pub.publish(skeletons);
    for (int i = 0; i<MAX_USERS; i++)
    {
      // decrease validity of skeleton whenever publish
      if (tracking_validity[i] > 0)
      {
        tracking_validity[i]--;
      }

      // eliminate invalid skeletons
      if (tracking_validity[i] == 0)
      {
        skeletons.data[i] = initialize_skeleton();
      }
    }

    showStatus();
    ros::spinOnce();
    loop_late.sleep();
  }

  return;
}

//-----------------------------------------------------------------------------
void SkeletonIntegrator::listenSkeletonStream()
{
  ros::Rate loop_late(10);
  std::vector<ros::Subscriber> sub(array.size());

  std::cout << "Subscribe ===" << std::endl;
  for (int i=0; i < array[i]; i++)
  {
    std::string topic_name("skeleton_stream_wrapper");
    sub[i] =nh.subscribe(topic_name.append(to_str<int>(array[i])),
        1,&SkeletonIntegrator::callback, this);
    std::cout << topic_name << std::endl;
  }

  ros::spin();

  return;
}

//-----------------------------------------------------------------------------
void SkeletonIntegrator::showStatus()
{
  std::stringstream ss;
  ss.fill(' ');

  ss << "New user: " << new_user_ << std::endl << std::endl;

  ss << "User ID" << std::endl;
  for (int i = 0; i < MAX_USERS; i++) { ss << "|" << std::setw(5) << i; }
  ss << "|" << std::endl;
  for (int i = 0; i < MAX_USERS; i++) { ss << "|" << std::setw(5) << skeletons.data[i].user_id; }
  ss << "|" << std::endl << std::endl;;

  ss << "Validity table" << std::endl;
  for (int i = 0; i < MAX_USERS; i++) { ss << "|" << std::setw(2) << i; }
  ss << "|" << std::endl;
  for (int i = 0; i < MAX_USERS; i++) { ss << "|" << std::setw(2) << tracking_validity[i]; }
  ss << "|" << std::endl << std::endl;

  ss << "Face state table" << std::endl;
  for (std::vector<int>::iterator it = array.begin(); it != array.end(); it++)
  {
    if (*it == 0) {continue;}
    ss << "|" << std::setw(2) << *it <<  "|";
    for (int i = 0; i < MAX_USERS; i++)
    {
      if (bFaceStateTable[i][*it])
      {
        ss << "T|";
      }
      else
      {
        ss << "F|";
      }
    }
    ss << std::endl;
  }

  ROS_INFO("%s", ss.str().c_str());
  return;
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv)
{
  int option;
  bool got_option = false;
  std::vector<int> cameraID_array;
  while ((option = getopt(argc,argv,"hn:"))!=-1)
  {
    switch(option)
    {
    case 'h':
      std::cout << "-- Usage" << std::endl <<
        std::endl;
      return 0;
      break;
    case 'n':
      for (int i = 0; i < atoi(optarg); i++)
      {
        cameraID_array.push_back(i+1);
      }
      got_option = true;
      break;
    case ':':
      std::cerr << "Need some values" << std::endl;
      break;
    case '?':
      std::cerr << "unknown option" << std::endl;
      break;
    }
  }
  if (!got_option)
  {
    for (int i = 1; i < argc; i++)
    {
      cameraID_array.push_back(atoi(argv[i]));
    }
  }

  int camera_num = cameraID_array.size();
  if (camera_num <= 0 || camera_num >= 7)
  {
    std::cout << "Maybe given invalid input." << std::endl;
    return -2;
  }
  ros::init(argc, argv, "skeleton_integrator");

  SkeletonIntegrator integrator(cameraID_array);

  integrator.run();

  return 0;
}
