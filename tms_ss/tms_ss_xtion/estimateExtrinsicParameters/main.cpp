#include <stdio.h>
#include <time.h>
#include <float.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstdint>

#include <unistd.h>

#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "MyCalibration.h"
#include "DepthImageMaker.h"
//#include "OpenNIDeviceListener.hpp"
#include "myutility.h"

extern Eigen::Vector2i g_points[4];

// Variables for debug
bool g_DebugFlg = false;
double d_roll, d_pitch, d_yaw;
double d_tx, d_ty, d_tz;
double d_convertion_threshold = 0.0001;
int d_convertion_method = 2;
int main (int argc, char **argv)
{
  int option;
  float pattern_width=45;
  while ((option=getopt(argc,argv,"dhw:0:12:"))!=-1)
  {
    char tmp = '\0';
    if (optarg == NULL)
    {
      optarg = &tmp;
    }
    std::string args(optarg);
    std::string delim(",");
    std::vector<std::string> list_args;
    switch(option)
    {
    case 'h':
      std::cout << "-- Usage" << std::endl <<
        "  -0\t" << "[roll],[pitch],[yaw],[tx],[ty],[tz]" << std::endl <<
        "  -1\t" << "Switch convertion method. (Perturbation method -> Steepest descent method)" << std::endl <<
        "  -2\t" << "Change threshold of convertion calcuration." << std::endl
        << std::endl;
      return 0;
      break;
    case 'd':
      g_DebugFlg = true;
      break;
    case 'w':
      pattern_width = atof(args.c_str());
    case '0':
      if (g_DebugFlg)
      {
        boost::split(list_args, args, boost::is_any_of(delim));
        d_roll = atof(list_args[0].c_str());
        d_pitch = atof(list_args[1].c_str());
        d_yaw = atof(list_args[2].c_str());
        d_tx = atof(list_args[3].c_str());
        d_ty = atof(list_args[4].c_str());
        d_tz = atof(list_args[5].c_str());
      }
      break;
    case '1':
      if (g_DebugFlg)
      {
        d_convertion_method = 1;
      }
      break;
    case '2':
      if (g_DebugFlg)
      {
        d_convertion_threshold = atof(args.c_str());
      }
      break;
    case ':':
      std::cerr << "Need some values" << std::endl;
      break;
    case '?':
      std::cerr << "unknown option" << std::endl;
      break;
    }
  }

  ros::init(argc, argv, "ExtrinsicParameterEstimation");
  ros::NodeHandle ros_nh;

  cv::namedWindow("Depth image");
  cv::setMouseCallback("Depth image", mouseCallback, 0);

  //For Debug
  g_points[0] << 295, 215;
  g_points[1] << 345, 215;
  g_points[2] << 295, 265;
  g_points[3] << 345, 265;

  pcl::visualization::CloudViewer viewer("OpenNI Viewer");

  // initialization
  char key;
  uint8_t *pdepth;
  uint8_t *pcolor;
  pdepth = new uint8_t [CAMERA_RESOLUTION_X*CAMERA_RESOLUTION_Y*2];
  pcolor = new uint8_t [CAMERA_RESOLUTION_X*CAMERA_RESOLUTION_Y*3];

  MyCalibration my_calib;
  my_calib.initialize(viewer, pdepth, pcolor);

  ros::Subscriber sub_depth = ros_nh.subscribe("/camera/depth/image_raw", 1, &MyCalibration::getDepthFrameCallback, &my_calib);
  ros::Subscriber sub_color = ros_nh.subscribe("/camera/rgb/image_raw", 1, &MyCalibration::getColorFrameCallback, &my_calib);

  ros::Rate loop_late(1000);
  ros::spin();

  KeyboardEventReader keyboard;

  while (ros::ok()) // key == 'q' ?
  {
    if (keyboard.getKeycode(key))
    {
      //printf("key:%c, %x\n", key, key);
      if (key == KEYCODE_r) // key == 'r' ?
      {
        //my_calib.extractPlanePoints();
        //  my_calib.viewPoints();
        //  my_calib.calcurateExtrinsicParameters(
        //      d_convertion_threshold,
        //      d_convertion_method,
        //      pattern_width);
      }
      if (key == KEYCODE_p) // key == 'p' ?
      {
        //my_calib.startPickingPoints();
      }
      if (key == KEYCODE_d) // key == 'd' ?
      {
        //my_calib.pickPointsAutomatically(5,6);
      }
    }
    ros::spinOnce();
  }

  //depth_stream.removeNewFrameListener(&new_frame_listener);

  //depth_stream.stop();
  //color_stream.stop();
  //depth_stream.destroy();
  //color_stream.destroy();
  //device.close();
  //openni::OpenNI::shutdown();

  return 0;
}
