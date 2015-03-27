#include <stdio.h>
#include <time.h>
#include <float.h>
#include <iostream>
#include <iomanip>
#include <sstream>

#include <unistd.h>

#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include "MyCalibration.h"
#include "DepthImageMaker.h"
#include "OpenNIDeviceListener.hpp"
#include "myutility.h"

extern Eigen::Vector2i g_points[4];

// pending ...
/*
void viewer_keyboard(const pcl::visualization::KeyboardEvent& event, void *args_void)
{
  if (event.keyDown())
  {
    std::vector<void*> &args = *(std::vector<void*>*)args_void;
    pcl::visualization::CloudViewer *viewer = (pcl::visualization::CloudViewer*)args[0];
    MyCalibration *y_calib = (MyCalibration*)args[1];
    if (event.getKeySym() == "a")
    {
      // Bug : This sentence stops program
      //my_calib->extractPlanePoints();
      //my_calib->viewPoints(*viewer);
    }
  }
  return;
}
*/

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

  openni::Status rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK)
  {
    printf("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());
    return 1;
  }

  openni::Array<openni::DeviceInfo> deviceList;
  openni::OpenNI::enumerateDevices(&deviceList);
  for (int i = 0; i < deviceList.getSize(); ++i)
  {
    printf("Device \"%s\" already connected\n", deviceList[i].getUri());
  }

  openni::Device device;
  rc = device.open(openni::ANY_DEVICE);
  if (rc != openni::STATUS_OK)
  {
    printf("Couldn't open device\n%s\n", openni::OpenNI::getExtendedError());
    return 2;
  }

  MyCalibration my_calib;
  DepthImageMaker new_frame_listener;

  OpenNIDeviceListener devicePrinter;

  openni::OpenNI::addDeviceConnectedListener(&devicePrinter);
  openni::OpenNI::addDeviceDisconnectedListener(&devicePrinter);
  openni::OpenNI::addDeviceStateChangedListener(&devicePrinter);


  //test
  openni::VideoStream depth_stream;
  openni::VideoStream color_stream;
  { // Create streams
    if (device.getSensorInfo(openni::SENSOR_DEPTH) != NULL)
    {
      rc = depth_stream.create(device, openni::SENSOR_DEPTH);
      if (rc != openni::STATUS_OK)
      {
        printf("Couldn't create depth_stream stream\n%s\n", openni::OpenNI::getExtendedError());
      }
    }

    if (device.getSensorInfo(openni::SENSOR_COLOR) != NULL)
    {
      rc = color_stream.create(device, openni::SENSOR_COLOR);
      if (rc != openni::STATUS_OK)
      {
        printf("Couldn't create colorstream\n%s\n", openni::OpenNI::getExtendedError());
      }
    }
  }

  std::vector<openni::VideoMode> config = getAllSensorInfo(device);
  depth_stream.setVideoMode(config[2]);
  color_stream.setVideoMode(config[1]);
  { // Mirroring off
    if (depth_stream.getMirroringEnabled())
    {
      depth_stream.setMirroringEnabled(false);
    }
    if (color_stream.getMirroringEnabled())
    {
      color_stream.setMirroringEnabled(false);
    }
  }

  { // Start stream
    rc = depth_stream.start();
    if (rc != openni::STATUS_OK)
    {
      printf("Couldn't start the depth_stream stream\n%s\n", openni::OpenNI::getExtendedError());
    }
    rc = color_stream.start();
    if (rc != openni::STATUS_OK)
    {
      printf("Couldn't start the color stream\n%s\n", openni::OpenNI::getExtendedError());
    }
  }

  cv::namedWindow("Depth image");
  cv::setMouseCallback("Depth image", mouseCallback, 0);

  depth_stream.addNewFrameListener(&new_frame_listener);

  //For Debug
  g_points[0] << 295, 215;
  g_points[1] << 345, 215;
  g_points[2] << 295, 265;
  g_points[3] << 345, 265;

  //pcl::visualization::PCLVisualizer viewer("OpenNI Viewer");
  //viewer.initCameraParameters();
  pcl::visualization::CloudViewer viewer("OpenNI Viewer");

  // initialization
  int key;
  my_calib.initialize(viewer, &depth_stream, &color_stream);
  std::vector<void*> args;
  args.push_back((void*)&viewer);
  args.push_back((void*)&my_calib);
  // pending ...
  //viewer.registerKeyboardCallback(viewer_keyboard, (void*)&args);

  while (key != 1048689) // key == 'q' ?
  {
    my_calib.image = new_frame_listener.image;
    if (key == 1048690) // key == 'r' ?
    {
      my_calib.extractPlanePoints();
      my_calib.viewPoints();
      my_calib.calcurateExtrinsicParameters(
          d_convertion_threshold,
          d_convertion_method,
          pattern_width);
    }
    if (key == 1048688) // key == 'p' ?
    {
      my_calib.startPickingPoints();
    }
    if (key == 1048676) // key == 'd' ?
    {
      my_calib.pickPointsAutomatically(5,6);
    }
    //std::cout << key << std::endl;
    key = cv::waitKey(100);
  }

  depth_stream.removeNewFrameListener(&new_frame_listener);

  depth_stream.stop();
  color_stream.stop();
  depth_stream.destroy();
  color_stream.destroy();
  device.close();
  openni::OpenNI::shutdown();

  return 0;
}
