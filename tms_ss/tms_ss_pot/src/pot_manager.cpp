//----------------------------------------------------------
// @file   : pot_manager.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.4 (since 2014.05.02)
// @date   : 2016.06.09
//----------------------------------------------------------

#include <ros/ros.h>
#include <pthread.h>
#include <std_msgs/String.h>
#include <tms_msg_ss/pot_tracking_points.h>
#include <tms_msg_ss/pot_tracking_grid.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <iostream>
#include <ctime>
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>

ros::Subscriber sub;

void msgSensingCallback(const tms_msg_ss::pot_tracking_points::ConstPtr &msg)
{
  std::cout << "start" << std::endl;
  if(msg->pot_tracking_grid.size() >0 ){
  std::cout << msg->pot_tracking_grid[0].id << std::endl;
   }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pot_manager");
    ros::NodeHandle nh;

    sub = nh.subscribe("tracking_point_inf", 1000, msgSensingCallback);
    ros::spin();
    return (0);
}


