
//------------------------------------------------------------------------------
// @file   : tms_db_state_publisher.cpp
// @brief  : subscribe the current information of object and publish the state of object
// @author : Yoonseok Pyo
// @version: Ver0.0.1 (since 2015.07.22)
// @date   : 2015.07.22
//------------------------------------------------------------------------------
//include for ROS
#include <ros/ros.h>
#include <unistd.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_db/Tmsdb.h>
#include <sensor_msgs/JointState.h>

//include for std
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <map>

//------------------------------------------------------------------------------
using std::string;
using std::vector;

//------------------------------------------------------------------------------
class DbStatePublisher
{
//------------------------------------------------------------------------------
private:
  // ROS NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // ROS Parameters
  bool is_debug;
  // MySQL structures
  // https://dev.mysql.com/doc/refman/5.6/en/c-api-data-structures.html

//------------------------------------------------------------------------------
public:
  // ROS Publisher
  tms_msg_db::TmsdbStamped current_environment_information;
  ros::Publisher state_pub;

  DbStatePublisher() :
    nh_priv("~"),
    is_debug(false)
  {
    //Init parameter
    nh_priv.param("is_debug", is_debug, is_debug);
    //Init Taget name
    ROS_ASSERT(initDbStatePublisher());

    state_pub = nh_priv.advertise<sensor_msgs::JointState>("db_state_publisher", 10);
  }

  //----------------------------------------------------------------------------
  ~DbStatePublisher()
  {
    ROS_ASSERT(shutdownDbStatePublisher());
  }

  //----------------------------------------------------------------------------
  void getDbCurrentInformation()
  {
    nh_priv.getParam("is_debug", is_debug);


    return;
  }

//------------------------------------------------------------------------------
private:
  bool initDbStatePublisher()
  {
    ROS_INFO("tms_db_state_publisher : Init OK!\n");
    return true;
  }

  //----------------------------------------------------------------------------
  bool shutdownDbStatePublisher()
  {
    //Close connection
    return true;
  }
};

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //Init ROS node
  ros::init(argc, argv, "tms_db_state_publisher");
  DbStatePublisher dp;
  ros::Rate loop_rate(100); // 0.01sec

  while (ros::ok())
  {
    // dp.getDbCurrentInformation();
    // dp.state_pub.publish(dp.current_environment_information);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

//------------------------------------------------------------------------------
//EOF
