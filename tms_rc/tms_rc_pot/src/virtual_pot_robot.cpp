//----------------------------------------------------------
// @file   : robot_virtual_test.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.4 (since 2015.08.05)
// @date   : 2015.08.05
//----------------------------------------------------------
#include "ros/ros.h"
#include "tms_msg_rc/robot_current_data.h"
#include "math.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "virtual_robot");
  ros::NodeHandle n;
  tms_msg_rc::robot_current_data plot;

  double x, y, z, yaw;
  double theta;

  ros::Publisher pub = n.advertise<tms_msg_rc::robot_current_data>("virtual", 1000);

  ros::Rate loop(100);
  while (ros::ok())
  {
    theta = theta + 0.001;
    yaw = yaw - 0.001;
    x = 1.0 * cos(theta);
    y = 1.0 * sin(theta);
    z = 0.2;

    plot.iID = 1;
    plot.fX = x;
    plot.fY = y;
    plot.fZ = z;
    plot.fTheta = theta;

    pub.publish(plot);
    loop.sleep();
  }
  return 0;
}
