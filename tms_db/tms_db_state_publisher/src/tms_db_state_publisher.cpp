
//------------------------------------------------------------------------------
// @file   : tms_db_state_publisher.cpp
// @brief  : subscribe the current information of object and publish the state of object
// @author : Yoonseok Pyo
// @version: Ver0.0.4 (since 2015.07.22)
// @date   : 2015.08.26
//------------------------------------------------------------------------------
//include for ROS
#include <ros/ros.h>
#include <unistd.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_db/Tmsdb.h>
#include <tms_msg_db/TmsdbGetData.h>
#include <sensor_msgs/JointState.h>

//include for std
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <map>
#include <boost/algorithm/string.hpp>

// #define rad2deg(x)	((x)*(180.0)/M_PI)
// #define deg2rad(x)	((x)*M_PI/180.0)

//------------------------------------------------------------------------------
using std::string;
using std::vector;
using namespace std;
using namespace boost;

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

  ros::ServiceClient get_data_client_;

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
    data_sub  = nh.subscribe("/tms_db_publisher", 1,  &DbStatePublisher::dbTFCallback, this);
    state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    get_data_client_ = nh.serviceClient<tms_msg_db::TmsdbGetData>("/tms_db_reader");
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
    double posX,posY,posZ,rotR,rotP,rotY;
    sensor_msgs::JointState state_data;
    std::string joint;

    if (msg->tmsdb.size()==0)
      return;

    for (uint32_t i=0; i<msg->tmsdb.size(); i++)
    {
      id      = msg->tmsdb[i].id;
      sensor  = msg->tmsdb[i].sensor;
      state   = msg->tmsdb[i].state;
      place   = msg->tmsdb[i].place;
      joint   = msg->tmsdb[i].joint;

      if (id==2003) // smartpal5-2
      {
        if (state==1)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotY = msg->tmsdb[i].ry;
          double g_jR[7] = {0.0,-0.08,0.0,0.0,0.0,0.0,0.0};
          double g_jL[7] = {0.0, 0.08,0.0,0.0,0.0,0.0,0.0};
          double g_gripper_right = -0.3;
          double g_gripper_left = 0.3;

          if(joint!=""){
            std::vector<std::string> v_joint;
            v_joint.clear();
            boost::split(v_joint,joint,boost::is_any_of(";"));
            std::stringstream ss;
            for(int i=0;i<7;i++){
              ss.clear();
              ss << v_joint.at(2+i);
              ss >> g_jR[i];
            }
            ss.clear();
            ss << v_joint.at(9);
            ss >> g_gripper_right;
            for(int i=0;i<7;i++){
              ss.clear();
              ss << v_joint.at(10+i);
              ss >> g_jL[i];
            }
            ss.clear();
            ss << v_joint.at(17);
            ss >> g_gripper_left;
          }

          state_data.header.stamp = ros::Time::now();
          state_data.name.push_back("smartpal5_x_joint");
          state_data.position.push_back(posX);
          state_data.name.push_back("smartpal5_y_joint");
          state_data.position.push_back(posY);
          state_data.name.push_back("smartpal5_yaw_joint");
          state_data.position.push_back(rotY);
          state_data.name.push_back("base_footprint_joint");
          state_data.position.push_back(0);
          state_data.name.push_back("lumbar_lower_joint");
          state_data.position.push_back(0);
          state_data.name.push_back("lumbar_upper_joint");
          state_data.position.push_back(0);
          // state_data.name.push_back("head_camera_joint");
          //state_data.position.push_back(0);
          state_data.name.push_back("l_arm_j1_joint");
          state_data.position.push_back(g_jL[0]);
          state_data.name.push_back("l_arm_j2_joint");
          state_data.position.push_back(g_jL[1]);//(0.08);
          state_data.name.push_back("l_arm_j3_joint");
          state_data.position.push_back(g_jL[2]);
          state_data.name.push_back("l_arm_j4_joint");
          state_data.position.push_back(g_jL[3]);
          state_data.name.push_back("l_arm_j5_joint");
          state_data.position.push_back(g_jL[4]);
          state_data.name.push_back("l_arm_j6_joint");
          state_data.position.push_back(g_jL[5]);
          state_data.name.push_back("l_arm_j7_joint");
          state_data.position.push_back(g_jL[6]);
          state_data.name.push_back("l_gripper_thumb_joint");
          state_data.position.push_back(g_gripper_left);
          // state_data.name.push_back("l_gripper_joint");
          // state_data.position.push_back(0);
          // state_data.name.push_back("l_end_effector_joint");
          // state_data.position.push_back(0);
          state_data.name.push_back("r_arm_j1_joint");
          state_data.position.push_back(g_jR[0]);
          state_data.name.push_back("r_arm_j2_joint");
          state_data.position.push_back(g_jR[1]);
          state_data.name.push_back("r_arm_j3_joint");
          state_data.position.push_back(g_jR[2]);
          state_data.name.push_back("r_arm_j4_joint");
          state_data.position.push_back(g_jR[3]);
          state_data.name.push_back("r_arm_j5_joint");
          state_data.position.push_back(g_jR[4]);
          state_data.name.push_back("r_arm_j6_joint");
          state_data.position.push_back(g_jR[5]);
          state_data.name.push_back("r_arm_j7_joint");
          state_data.position.push_back(g_jR[6]);
          state_data.name.push_back("r_gripper_thumb_joint");
          state_data.position.push_back(g_gripper_right);
          // state_data.name.push_back("r_gripper_joint");
          // state_data.position.push_back(0);
          // state_data.name.push_back("r_end_effector_joint");
          // state_data.position.push_back(0);
        }
      }

      if (id==2009) // refrigerator
      {
        if (state!=0)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotY = msg->tmsdb[i].ry;

          if(posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("refrigerator_x_joint");
            state_data.name.push_back("refrigerator_y_joint");
            state_data.name.push_back("refrigerator_yaw_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(rotY);
          }
        }
      }

      if (id==6004) // chair
      {
        if (state!=0)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotY = msg->tmsdb[i].ry;

          if(posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("chair_x_joint");
            state_data.name.push_back("chair_y_joint");
            state_data.name.push_back("chair_yaw_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(rotY);
          }
        }
      }

      if (id==6007) // meeting_chair1
      {
        if (state!=0)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotY = msg->tmsdb[i].ry;

          if(posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("meeting_chair1_x_joint");
            state_data.name.push_back("meeting_chair1_y_joint");
            state_data.name.push_back("meeting_chair1_yaw_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(rotY);
          }
        }
      }

      if (id==6008) // meeting_chair2
      {
        if (state!=0)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotY = msg->tmsdb[i].ry;

          if(posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("meeting_chair2_x_joint");
            state_data.name.push_back("meeting_chair2_y_joint");
            state_data.name.push_back("meeting_chair2_yaw_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(rotY);
          }
        }
      }

      if (id==6009) // meeting_chair3
      {
        if (state!=0)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotY = msg->tmsdb[i].ry;

          if(posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("meeting_chair3_x_joint");
            state_data.name.push_back("meeting_chair3_y_joint");
            state_data.name.push_back("meeting_chair3_yaw_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(rotY);
          }
        }
      }

      if (id==6010) // meeting_chair4
      {
        if (state!=0)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotY = msg->tmsdb[i].ry;

          if(posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("meeting_chair4_x_joint");
            state_data.name.push_back("meeting_chair4_y_joint");
            state_data.name.push_back("meeting_chair4_yaw_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(rotY);
          }
        }
      }

      if (id==6018) // wagon
      {
        if (state==1)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotY = msg->tmsdb[i].ry;

          if(posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("wagon_x_joint");
            state_data.name.push_back("wagon_y_joint");
            state_data.name.push_back("wagon_yaw_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(rotY);
          }
        }
      }

      if(id==7001) //chipstar_red
      {
        if(state != 0)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          posZ = msg->tmsdb[i].z;
          rotR = msg->tmsdb[i].rr;
          rotP = msg->tmsdb[i].rp;
          rotY = msg->tmsdb[i].ry;

          if(posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else
          {
            state_data.header.stamp = ros::Time::now();
            state_data.name.push_back("chipstar_red_x_joint");
            state_data.name.push_back("chipstar_red_y_joint");
            state_data.name.push_back("chipstar_red_z_joint");
            state_data.name.push_back("chipstar_red_yaw_joint");
            state_data.name.push_back("chipstar_red_pitch_joint");
            state_data.name.push_back("chipstar_red_roll_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(posZ);
            state_data.position.push_back(rotY);
            state_data.position.push_back(rotP);
            state_data.position.push_back(rotR);
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
