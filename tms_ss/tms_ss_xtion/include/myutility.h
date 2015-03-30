#ifndef _MY_UTILITY_H_
#define _MY_UTILITY_H_

#include <opencv2/opencv.hpp>

const std::vector<openni::VideoMode> getAllSensorInfo(openni::Device& device);
void showVideoMode(const openni::VideoMode& video_mode);
int find_corner(const cv::Mat& image, const cv::Size pattern_size, 
    std::vector<cv::Point2f>& corners, bool& pattern_found);

#endif
