#include <iostream>
#include <string>
#include <sstream>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <tms_ss_kinect_v2/SkeletonArray.h>
#include <tms_ss_kinect_v2/SkeletonStreamWrapper.h>

#include <unistd.h>

template <class T>
std::string to_str(const T& t)
{
  std::stringstream ss;
  ss << t;
  return ss.str();
}

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

class SkeletonIntegrator
{
  public:
    SkeletonIntegrator(const std::vector<int>& camera);
    ~SkeletonIntegrator();

    void callback(const tms_ss_kinect_v2::SkeletonStreamWrapper::ConstPtr& msg);
    void run();
  private:
    ros::NodeHandle nh;
    ros::Publisher* ppub;
    std::vector<int> array;
    tms_ss_kinect_v2::SkeletonArray skeletons;
};

SkeletonIntegrator::SkeletonIntegrator(const std::vector<int> &camera) :
  array(camera)
{
	skeletons.data.resize(6);
	for (int i = 0; i < 6; i++)
	{
		skeletons.data[i] = initialize_skeleton();
	}
  return;
}

SkeletonIntegrator::~SkeletonIntegrator()
{
  return;
}

void SkeletonIntegrator::callback(const tms_ss_kinect_v2::SkeletonStreamWrapper::ConstPtr& msg)
{
  ROS_INFO("Received skeleton from camera %d", msg->camera_number);

  tms_ss_kinect_v2::Skeleton skeleton = msg->skeleton;
  tms_ss_kinect_v2::CameraPosture camera_posture = msg->camera_posture;

  Eigen::Vector3f translation(
      camera_posture.translation.x,
      camera_posture.translation.y,
      camera_posture.translation.z);
  Eigen::Quaternionf rotation(
      camera_posture.rotation.w,
      camera_posture.rotation.x,
      camera_posture.rotation.y,
      camera_posture.rotation.z);

  tms_ss_kinect_v2::Skeleton integrated_skeleton;
	integrated_skeleton.user_id = skeleton.user_id;
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
	// Temporary 
	skeletons.data.resize(6);
	skeletons.data[integrated_skeleton.user_id] = integrated_skeleton;
  //for (int i = 0; i < skeletons.data.size(); i++)
  //{
  //  //Eigen::Vector3f v1(
  //  //    integrated_skeleton.position[0].x,
  //  //    integrated_skeleton.position[0].y,
  //  //    integrated_skeleton.position[0].z);
  //  //Eigen::Vector3f v2(
  //  //    skeletons.data[i].position[0].x,
  //  //    skeletons.data[i].position[0].y,
  //  //    skeletons.data[i].position[0].z);
  //  //if ((v1-v2).norm() < 0.3)
  //  //{
  //  //  already_detect = true;
  //  //  skeletons.data[i] = integrated_skeleton;
  //  //}
  //  //std::cout << (v1-v2).norm() << std::endl;
  //}
  //if (!already_detect)
  //{
  //  skeletons.data.push_back(integrated_skeleton);
  //}
	
	int skeleton_num = skeletons.data.size();
	std::cout << skeletons.data.size();
	if (skeleton_num > 1)
	{
		std::cout << " skeletons are detected." << std::endl;
	}
	else
	{
		std::cout << " skeletons is detected." << std::endl;
	}
  ppub->publish(skeletons);

  return;
}

void SkeletonIntegrator::run()
{
  std::vector<ros::Subscriber> sub(array.size());
  std::cout << "Subscribe ===" << std::endl;
  for (int i=0; i < array[i]; i++)
  {
    std::string topic_name("skeleton_stream_wrapper");
    sub[i] =nh.subscribe(topic_name.append(to_str(array[i])),
        10,&SkeletonIntegrator::callback, this);
    std::cout << topic_name << std::endl;
  }
  ros::Publisher pub =
    nh.advertise<tms_ss_kinect_v2::SkeletonArray>(
        "integrated_skeleton_stream", 1);
  ppub = &pub;
  ros::spin();
  return;
}

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
