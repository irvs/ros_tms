#include <iostream>
#include <vector>
#include <OpenNI.h>

#ifdef __linux__
#include <termios.h>
#include <unistd.h>
#endif

#include "myutility.h"

/**--------------------------------------------------
 * \class KeyboardEventReader
 */
KeyboardEventReader::KeyboardEventReader()
{
  kfd = 0;
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);

  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
}

KeyboardEventReader::~KeyboardEventReader()
{
  tcsetattr(kfd, TCSANOW, &cooked);
}

bool KeyboardEventReader::getKeycode(char &c)
{
  if(read(kfd, &c, 1) < 0)
  {
    perror("read():");
    return false;
  }
  return true;
}
/*---------------------------------------------------*/

void showVideoMode(const openni::VideoMode& video_mode)
{
  std::cout <<
    "Fps         : " << video_mode.getFps() << "  "
    "Pixel format: " << video_mode.getPixelFormat() << "  " <<
    "Resolution X: " << video_mode.getResolutionX() << "  " <<
    "Resolution Y: " << video_mode.getResolutionY() <<
    std::endl;
  return;
}

const std::vector<openni::VideoMode> getAllSensorInfo(openni::Device& device)
{
  int i;
  const openni::SensorInfo *sensor;
  std::vector<openni::VideoMode> ret;
  if (device.hasSensor(openni::SENSOR_IR))
  {
    std::cout << "Information about IR Sensor" << std::endl;
    sensor = device.getSensorInfo(openni::SENSOR_IR);
    const openni::Array<openni::VideoMode>& supported_video_modes = sensor->getSupportedVideoModes();
    for (i = 0; i < supported_video_modes.getSize(); i++)
    {
      const openni::VideoMode& vm = supported_video_modes[i];
      if (vm.getFps() == 30 &&
          vm.getPixelFormat() == openni::PIXEL_FORMAT_GRAY16 &&
          vm.getResolutionX() == 640 &&
          vm.getResolutionY() == 480)
      {
        showVideoMode(supported_video_modes[i]);
        ret.push_back(supported_video_modes[i]);
      }
    }
  }
  if (device.hasSensor(openni::SENSOR_COLOR))
  {
    std::cout << "Information about COLOR Sensor" << std::endl;
    sensor = device.getSensorInfo(openni::SENSOR_COLOR);
    const openni::Array<openni::VideoMode>& supported_video_modes = sensor->getSupportedVideoModes();
    for (i = 0; i < supported_video_modes.getSize(); i++)
    {
      const openni::VideoMode& vm = supported_video_modes[i];
      if (vm.getFps() == 30 &&
          vm.getPixelFormat() == openni::PIXEL_FORMAT_RGB888 &&
          vm.getResolutionX() == 640 &&
          vm.getResolutionY() == 480)
      {
        showVideoMode(supported_video_modes[i]);
        ret.push_back(supported_video_modes[i]);
      }
    }
  }
  if (device.hasSensor(openni::SENSOR_DEPTH))
  {
    std::cout << "Information about DEPTH Sensor" << std::endl;
    sensor = device.getSensorInfo(openni::SENSOR_DEPTH);
    const openni::Array<openni::VideoMode>& supported_video_modes =
      sensor->getSupportedVideoModes();
    for (i = 0; i < supported_video_modes.getSize(); i++)
    {
      const openni::VideoMode& vm = supported_video_modes[i];
      if (vm.getFps() == 30 &&
          vm.getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM &&
          vm.getResolutionX() == 640 &&
          vm.getResolutionY() == 480)
      {
        showVideoMode(supported_video_modes[i]);
        ret.push_back(supported_video_modes[i]);
      }
    }
  }
  return ret;
}

int find_corner(const cv::Mat& image, const cv::Size pattern_size, 
    std::vector<cv::Point2f>& corners, bool& pattern_found)
{
  cv::Mat gray(image.rows, image.cols, CV_8UC1);
  switch(image.type())
  {
  case CV_8UC1:
    gray = image;
    break;
  case CV_8UC3:
    cv::cvtColor(image, gray, CV_BGR2GRAY);
    break;
  default:
    std::cerr << "Unexpected image type." << std::endl;
    return -1;
  }
  pattern_found = cv::findChessboardCorners(gray, pattern_size, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
  if (pattern_found)
  {
    cv::cornerSubPix(gray, corners, cv::Size(15,15), cv::Size(-1,-1),
        cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 15, 0.01));
  }
  else
  {
    //std::cerr << "Warning: Couldn't detect corner" << std::endl;
  }
  return 0;
}
