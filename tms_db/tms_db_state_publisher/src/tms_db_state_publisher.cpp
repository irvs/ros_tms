
//------------------------------------------------------------------------------
// @file   : tms_db_state_publisher.cpp
// @brief  : subscribe the current information of object and publish the state of object
// @author : Yoonseok Pyo
// @version: Ver0.0.2 (since 2015.07.22)
// @date   : 2015.07.23
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

#define rad2deg(x)	((x)*(180.0)/M_PI)
#define deg2rad(x)	((x)*M_PI/180.0)

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
  // ROS Topic Subscriber
  ros::Subscriber data_sub;
  // ROS Topic Publisher
  ros::Publisher  state_pub;  

//------------------------------------------------------------------------------
public:
  DbStatePublisher() :
    nh_priv("~"),
    is_debug(false)
  {
    //Init parameter
    nh_priv.param("is_debug", is_debug, is_debug);
    //Init target name
    ROS_ASSERT(initDbStatePublisher());
    data_sub  = nh.subscribe("/tms_db_publisher/db_publisher", 1,  &DbStatePublisher::dbTFCallback, this);
    state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
  }

  //----------------------------------------------------------------------------
  ~DbStatePublisher()
  {
    ROS_ASSERT(shutdownDbStatePublisher());
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
    return true;
  }

  //----------------------------------------------------------------------------
  // void dbTFCallback(const ros::MessageEvent<tms_msg_db::TmsdbStamped const>& event)
  void dbTFCallback(const tms_msg_db::TmsdbStamped::ConstPtr& msg)
  {
    uint32_t id, oID, sensor, state, place;
    double posX,posY,posT;
    sensor_msgs::JointState state_data;

    if (msg->tmsdb.size()==0)
      return;

    for (uint32_t i=0; i<msg->tmsdb.size(); i++)
    {
      id      = msg->tmsdb[i].id;
      sensor  = msg->tmsdb[i].sensor;
      state   = msg->tmsdb[i].state;
      place   = msg->tmsdb[i].place;

      if (id ==6019) // wagon
      {
        if (state==1)
        {
          posX = msg->tmsdb[i].x/1000;
          posY = msg->tmsdb[i].y/1000;
          posT = deg2rad(msg->tmsdb[i].ry);

          if(posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("wagon_x_joint");
            state_data.name.push_back("wagon_y_joint");
            state_data.name.push_back("wagon_theta_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(posT);
          }
        }
      }
    }
    state_pub.publish(state_data);
  }  
};

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //Init ROS node
  ros::init(argc, argv, "tms_db_state_publisher");
  DbStatePublisher dsp;
  ros::spin();
  return 0;
}

//------------------------------------------------------------------------------
//EOF
