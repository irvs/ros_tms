
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

// LRF_POSITIONS----------
#define LRF1_X 0.0
#define LRF1_Y 0.0
#define LRF1_ANGLE 0.0
#define LRF2_X 4.5
#define LRF2_Y 2.0
#define LRF2_ANGLE PI
#define LRF3_X 4.0
#define LRF3_Y 9.5
#define LRF3_ANGLE PI
#define LRF4_X 1.75
#define LRF4_Y 9.5
#define LRF4_ANGLE 0.0
//----------------------

ros::Publisher  pub1,pub2,pub3,pub4;
ros::Subscriber sub1,sub2,sub3,sub4;

double theta = 0.0;

// float colorset[14][4] = {{1, 0, 0, 0},
//                          {0, 1, 0, 0},
//                          {0, 0, 1, 0},
//                          {1, 1, 0, 0},
//                          {0, 1, 1, 0},
//                          {1, 0, 1, 0},
//                          {1, 1, 1, 0},
//                          {0.5, 0, 0, 0},
//                          {0, 0.5, 0, 0},
//                          {0, 0, 0.5, 0},
//                          {0.5, 0.5, 0, 0},
//                          {0, 0.5, 0.5, 0},
//                          {0.5, 0, 0.5, 0},
//                          {0.5, 0.5, 0.5, 0}};

void laser_visualization_callback1(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  visualization_msgs::MarkerArray markerArray;

  double laser_x     = LRF1_X;
  double laser_y     = LRF1_Y;
  double laser_theta = LRF1_ANGLE;
  double laser_point_x;
  double laser_point_y;

  CvMat *m_Rotate = cvCreateMat(2, 2, CV_64F);
  CvMat *Temp = cvCreateMat(2, 1, CV_64F);
  CvMat *Temp2 = cvCreateMat(2, 1, CV_64F);
  double theta;

  cvmSet(m_Rotate, 0, 0,  cos(laser_theta));
  cvmSet(m_Rotate, 0, 1, -sin(laser_theta));
  cvmSet(m_Rotate, 1, 0,  sin(laser_theta));
  cvmSet(m_Rotate, 1, 1,  cos(laser_theta));

  int id = 0;

  double prev_x, prev_y;

  for (unsigned int i = 0; i < scan->ranges.size(); i++)
  {
    theta = scan->angle_increment * i + scan->angle_min;
    cvmSet(Temp, 0, 0, scan->ranges[i] * cos(theta));
    cvmSet(Temp, 1, 0, scan->ranges[i] * sin(theta));
    cvMatMul(m_Rotate, Temp, Temp2);
    laser_point_x = cvmGet(Temp2, 0, 0);
    laser_point_y = cvmGet(Temp2, 1, 0);
    laser_point_x = laser_point_x + laser_x;
    laser_point_y = laser_point_y + laser_y;

    if(i == 0) {
       prev_x = laser_point_x;
       prev_y = laser_point_y;
    }

    visualization_msgs::Marker marker;
    uint32_t laser_line = visualization_msgs::Marker::LINE_STRIP;

    marker.header.frame_id = "/world_link";
    marker.header.stamp = ros::Time::now();
    marker.id = id;
    id++;
    marker.type = laser_line;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.color.a = 0.5;
    geometry_msgs::Point ps;
/*
    ps.x = laser_x;
    ps.y = laser_y;
*/
    ps.x = prev_x;
    ps.y = prev_y;
    ps.z = 1.0;
    marker.points.push_back(ps);

    ps.x = laser_point_x;
    ps.y = laser_point_y;
    ps.z = 1.0;
    marker.points.push_back(ps);

    marker.lifetime = ros::Duration(1.1);
    markerArray.markers.push_back(marker);

    prev_x = laser_point_x;
    prev_y = laser_point_y;

  }
  pub1.publish(markerArray);
}

void laser_visualization_callback2(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  visualization_msgs::MarkerArray markerArray;

  double laser_x     = LRF2_X;
  double laser_y     = LRF2_Y;
  double laser_theta = LRF2_ANGLE;
  double laser_point_x;
  double laser_point_y;

  CvMat *m_Rotate = cvCreateMat(2, 2, CV_64F);
  CvMat *Temp = cvCreateMat(2, 1, CV_64F);
  CvMat *Temp2 = cvCreateMat(2, 1, CV_64F);
  double theta;

  cvmSet(m_Rotate, 0, 0,  cos(laser_theta));
  cvmSet(m_Rotate, 0, 1, -sin(laser_theta));
  cvmSet(m_Rotate, 1, 0,  sin(laser_theta));
  cvmSet(m_Rotate, 1, 1,  cos(laser_theta));

  int id = 0;

  double prev_x, prev_y;

  for (unsigned int i = 0; i < scan->ranges.size(); i++)
  {
    theta = scan->angle_increment * i + scan->angle_min;
    cvmSet(Temp, 0, 0, scan->ranges[i] * cos(theta));
    cvmSet(Temp, 1, 0, scan->ranges[i] * sin(theta));
    cvMatMul(m_Rotate, Temp, Temp2);
    laser_point_x = cvmGet(Temp2, 0, 0);
    laser_point_y = cvmGet(Temp2, 1, 0);
    laser_point_x = laser_point_x + laser_x;
    laser_point_y = laser_point_y + laser_y;

    visualization_msgs::Marker marker;
    uint32_t laser_line = visualization_msgs::Marker::LINE_STRIP;

    marker.header.frame_id = "/world_link";
    marker.header.stamp = ros::Time::now();
    marker.id = id;
    id++;
    marker.type = laser_line;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 0.5;
    geometry_msgs::Point ps;
 /*
    ps.x = laser_x;
    ps.y = laser_y;
*/
    ps.x = prev_x;
    ps.y = prev_y;
    ps.z = 1.0;
    marker.points.push_back(ps);

    ps.x = laser_point_x;
    ps.y = laser_point_y;
    ps.z = 1.0;
    marker.points.push_back(ps);

    marker.lifetime = ros::Duration(1.1);
    markerArray.markers.push_back(marker);

    prev_x = laser_point_x;
    prev_y = laser_point_y;

  }
  pub2.publish(markerArray);
}

void laser_visualization_callback3(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  visualization_msgs::MarkerArray markerArray;

  double laser_x     = LRF3_X;
  double laser_y     = LRF3_Y;
  double laser_theta = LRF3_ANGLE;
  double laser_point_x;
  double laser_point_y;

  CvMat *m_Rotate = cvCreateMat(2, 2, CV_64F);
  CvMat *Temp = cvCreateMat(2, 1, CV_64F);
  CvMat *Temp2 = cvCreateMat(2, 1, CV_64F);
  double theta;

  cvmSet(m_Rotate, 0, 0,  cos(laser_theta));
  cvmSet(m_Rotate, 0, 1, -sin(laser_theta));
  cvmSet(m_Rotate, 1, 0,  sin(laser_theta));
  cvmSet(m_Rotate, 1, 1,  cos(laser_theta));

  int id = 0;

  double prev_x, prev_y;

  for (unsigned int i = 0; i < scan->ranges.size(); i++)
  {
    theta = scan->angle_increment * i + scan->angle_min;
    cvmSet(Temp, 0, 0, scan->ranges[i] * cos(theta));
    cvmSet(Temp, 1, 0, scan->ranges[i] * sin(theta));
    cvMatMul(m_Rotate, Temp, Temp2);
    laser_point_x = cvmGet(Temp2, 0, 0);
    laser_point_y = cvmGet(Temp2, 1, 0);
    laser_point_x = laser_point_x + laser_x;
    laser_point_y = laser_point_y + laser_y;

    visualization_msgs::Marker marker;
    uint32_t laser_line = visualization_msgs::Marker::LINE_STRIP;

    marker.header.frame_id = "/world_link";
    marker.header.stamp = ros::Time::now();
    marker.id = id;
    id++;
    marker.type = laser_line;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 0.5;
    geometry_msgs::Point ps;
/*
    ps.x = laser_x;
    ps.y = laser_y;
*/
    ps.x = prev_x;
    ps.y = prev_y;
    ps.z = 1.0;
    marker.points.push_back(ps);

    ps.x = laser_point_x;
    ps.y = laser_point_y;
    ps.z = 1.0;
    marker.points.push_back(ps);

    marker.lifetime = ros::Duration(1.1);
    markerArray.markers.push_back(marker);

    prev_x = laser_point_x;
    prev_y = laser_point_y;

  }
  pub3.publish(markerArray);
}

void laser_visualization_callback4(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  visualization_msgs::MarkerArray markerArray;

  double laser_x     = LRF4_X;
  double laser_y     = LRF4_Y;
  double laser_theta = LRF4_ANGLE;
  double laser_point_x;
  double laser_point_y;

  CvMat *m_Rotate = cvCreateMat(2, 2, CV_64F);
  CvMat *Temp = cvCreateMat(2, 1, CV_64F);
  CvMat *Temp2 = cvCreateMat(2, 1, CV_64F);
  double theta;

  cvmSet(m_Rotate, 0, 0,  cos(laser_theta));
  cvmSet(m_Rotate, 0, 1, -sin(laser_theta));
  cvmSet(m_Rotate, 1, 0,  sin(laser_theta));
  cvmSet(m_Rotate, 1, 1,  cos(laser_theta));

  int id = 0;

  double prev_x, prev_y;

  for (unsigned int i = 0; i < scan->ranges.size(); i++)
  {
    theta = scan->angle_increment * i + scan->angle_min;
    cvmSet(Temp, 0, 0, scan->ranges[i] * cos(theta));
    cvmSet(Temp, 1, 0, scan->ranges[i] * sin(theta));
    cvMatMul(m_Rotate, Temp, Temp2);
    laser_point_x = cvmGet(Temp2, 0, 0);
    laser_point_y = cvmGet(Temp2, 1, 0);
    laser_point_x = laser_point_x + laser_x;
    laser_point_y = laser_point_y + laser_y;

    visualization_msgs::Marker marker;
    uint32_t laser_line = visualization_msgs::Marker::LINE_STRIP;

    marker.header.frame_id = "/world_link";
    marker.header.stamp = ros::Time::now();
    marker.id = id;
    id++;
    marker.type = laser_line;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 0.5;
    geometry_msgs::Point ps;
/*
    ps.x = laser_x;
    ps.y = laser_y;
*/
    ps.x = prev_x;
    ps.y = prev_y;
    ps.z = 1.0;
    marker.points.push_back(ps);

    ps.x = laser_point_x;
    ps.y = laser_point_y;
    ps.z = 1.0;
    marker.points.push_back(ps);

    marker.lifetime = ros::Duration(1.1);
    markerArray.markers.push_back(marker);

    prev_x = laser_point_x;
    prev_y = laser_point_y;

  }
  pub4.publish(markerArray);
}

// main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_visualization");
  ros::NodeHandle n;
  pub1 = n.advertise< visualization_msgs::MarkerArray >("laser_visualization1", 1);
  pub2 = n.advertise< visualization_msgs::MarkerArray >("laser_visualization2", 1);
  pub3 = n.advertise< visualization_msgs::MarkerArray >("laser_visualization3", 1);
  pub4 = n.advertise< visualization_msgs::MarkerArray >("laser_visualization4", 1);
  sub1 = n.subscribe("/LaserTracker1", 1, laser_visualization_callback1);
  sub2 = n.subscribe("/LaserTracker2", 1, laser_visualization_callback2);
  sub3 = n.subscribe("/LaserTracker3", 1, laser_visualization_callback3);
  sub4 = n.subscribe("/LaserTracker4", 1, laser_visualization_callback4);
  ros::spin();
  return 0;
}
