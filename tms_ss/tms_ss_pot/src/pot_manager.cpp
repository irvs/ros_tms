//----------------------------------------------------------
// @file   : pot_manager.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.4 (since 2014.05.02)
// @date   : 2016.11.14
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
	ROS_INFO_STREAM("msgSensingCallback #s");
    //std::cout << "start" << std::endl;
    if (msg->pot_tracking_grid.size() > 0)
    {
        std::cout << "id " << msg->pot_tracking_grid[0].id << std::endl;
        std::cout << "flag " << msg->pot_tracking_grid[0].flag << std::endl;
        std::cout << "count "  << msg->pot_tracking_grid[0].count << std::endl;
        std::cout << "startx "  << msg->pot_tracking_grid[0].start_x << std::endl;
        std::cout << "starty "  << msg->pot_tracking_grid[0].start_y << std::endl;
        std::cout << "tmp_x "  << msg->pot_tracking_grid[0].tmp_x << std::endl;
        std::cout << "tmp_y "  << msg->pot_tracking_grid[0].tmp_y << std::endl;
        std::cout << "end_x "  << msg->pot_tracking_grid[0].end_x << std::endl;
        std::cout << "end_y "  << msg->pot_tracking_grid[0].end_y << std::endl;
        std::cout << "vect_x "  << msg->pot_tracking_grid[0].vector_x << std::endl;
        std::cout << "vect_y "  << msg->pot_tracking_grid[0].vector_y << std::endl;
    }
    ROS_INFO_STREAM("msgSensingCallback #e");

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pot_manager");
    ros::NodeHandle nh;

    sub = nh.subscribe("tracking_point_inf", 1000, msgSensingCallback);
    ros::spin();
    return (0);
}


