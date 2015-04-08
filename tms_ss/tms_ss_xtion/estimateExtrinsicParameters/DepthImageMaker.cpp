#include <OpenNI.h>
#include <opencv2/opencv.hpp>

#include "DepthImageMaker.h"

void DepthImageMaker::onNewFrame(openni::VideoStream& stream)
{
  stream.readFrame(&frame);
  if (frame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM
      && frame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_100_UM)
  {
    std::cerr << "Unexpected frame format\n" << std::endl;
    return;
  }
  cv::Mat image_tmp = cv::Mat(stream.getVideoMode().getResolutionY(),
      stream.getVideoMode().getResolutionX(),
      CV_16UC1, (char*)frame.getData());
  image_tmp.convertTo(image, CV_8UC1, 255.0/10000.0);
  return;
}
