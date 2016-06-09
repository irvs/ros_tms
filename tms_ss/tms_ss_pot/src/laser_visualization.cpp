//------------------------------------------------------------------------------
// @file   : laser_visualization.cpp
// @brief  : visulaize function
// @author : Watanabe Yuuta
// @version: Ver0.1.1
// @date   : 2015.07.31
//------------------------------------------------------------------------------
#include "ros/ros.h"
#include "stdio.h"
#include "tms_msg_ss/tracking_points.h"
#include "tms_msg_ss/tracking_grid.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "iostream"
#include "math.h"
#include <sensor_msgs/LaserScan.h>
#include "opencv/cv.h"

#define PI 3.1415926535897932384626433832795

ros::Publisher pub;
ros::Subscriber sub;

double theta = 0.0;

// colorset
float colorset[14][4] = {{1, 0, 0, 0},
                         {0, 1, 0, 0},
                         {0, 0, 1, 0},
                         {1, 1, 0, 0},
                         {0, 1, 1, 0},
                         {1, 0, 1, 0},
                         {1, 1, 1, 0},
                         {0.5, 0, 0, 0},
                         {0, 0.5, 0, 0},
                         {0, 0, 0.5, 0},
                         {0.5, 0.5, 0, 0},
                         {0, 0.5, 0.5, 0},
                         {0.5, 0, 0.5, 0},
                         {0.5, 0.5, 0.5, 0}};

void laser_visualization_callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  /*
  sensor_msgs::LaserScan Laser;
  Laser.header.seq      = scan->header.seq;
  Laser.header.stamp    = scan->header.stamp;
  Laser.header.frame_id = scan->header.frame_id;
  Laser.angle_min       = scan->angle_min;
  Laser.angle_max       = scan->angle_max;
  Laser.angle_increment = scan->angle_increment;
  Laser.time_increment  = scan->time_increment;
  Laser.scan_time       = scan->scan_time;
  Laser.range_min       = scan->range_min;
  Laser.range_max       = scan->range_max;
  Laser.ranges          = scan->ranges;
  */

  visualization_msgs::MarkerArray markerArray;

  double laser_x = 7.75;
  double laser_y = 5.55;
  double laser_point_x;
  double laser_point_y;

  CvMat *m_Rotate = cvCreateMat(2, 2, CV_64F);
  CvMat *Temp = cvCreateMat(2, 1, CV_64F);
  CvMat *Temp2 = cvCreateMat(2, 1, CV_64F);
  double theta;

  cvmSet(m_Rotate, 0, 0, cos(-PI / 2.0));
  cvmSet(m_Rotate, 0, 1, -sin(-PI / 2.0));
  cvmSet(m_Rotate, 1, 0, sin(-PI / 2.0));
  cvmSet(m_Rotate, 1, 1, cos(-PI / 2.0));

  int id = 0;
  for (unsigned int i = 0; i < scan->ranges.size(); i++)
  {
    theta = scan->angle_increment * i + scan->angle_min;
    cvmSet(Temp, 0, 0, scan->ranges[i] * cos(theta));
    cvmSet(Temp, 1, 0, scan->ranges[i] * sin(theta));
    cvmMul(m_Rotate, Temp, Temp2);
    laser_point_x = cvmGet(Temp2, 0, 0);
    laser_point_y = cvmGet(Temp2, 1, 0);
    laser_point_x = laser_point_x + laser_x;
    laser_point_y = laser_point_y + laser_y;
    // add draw_line

    visualization_msgs::Marker marker;
    uint32_t laser_line = visualization_msgs::Marker::LINE_STRIP;

    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();
    marker.id = id;
    id++;
    marker.type = laser_line;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.005;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.color.a = 0.5;
    geometry_msgs::Point ps;
    ps.x = laser_x;
    ps.y = laser_y;
    ps.z = 1.0;
    marker.points.push_back(ps);

    ps.x = laser_point_x;
    ps.y = laser_point_y;
    ps.z = 1.0;
    // std::cout << id << " " << laser_x << " " << laser_y <<  " "  << laser_point_x << " " << laser_point_y <<
    // std::endl;
    std::cout << scan->ranges.size() << std::endl;
    marker.points.push_back(ps);

    marker.lifetime = ros::Duration(0.02);
    markerArray.markers.push_back(marker);
  }
  pub.publish(markerArray);
}

// main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_visualization");
  ros::NodeHandle n;
  pub = n.advertise<visualization_msgs::MarkerArray>("laser_visualization", 1);
  sub = n.subscribe("/LaserTracker1", 1, laser_visualization_callback);
  ros::spin();
  return 0;
}
