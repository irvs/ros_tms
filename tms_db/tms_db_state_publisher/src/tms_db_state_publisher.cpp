
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
#include <tms_msg_ss/SkeletonArray.h>

//include for std
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <map>
#include <boost/algorithm/string.hpp>
#include <tf/transform_broadcaster.h>

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
  ros::Publisher  skeleton_pub;

  ros::ServiceClient get_data_client_;
  tf::TransformBroadcaster br;
  tf::Transform transform;

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
    skeleton_pub = nh.advertise<tms_msg_ss::SkeletonArray>("integrated_skeleton_stream", 1);
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

    bool type;
    nh.getParam("/is_real",type);

    for (uint32_t i=0; i<msg->tmsdb.size(); i++)
    {
      id      = msg->tmsdb[i].id;
      sensor  = msg->tmsdb[i].sensor;
      state   = msg->tmsdb[i].state;
      place   = msg->tmsdb[i].place;
      joint   = msg->tmsdb[i].joint;

      if (id==2003 && ((type==true && sensor==3003)||(type==false && sensor==3005))) // smartpal5-2
      {
        if (state==1)
        {
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotY = msg->tmsdb[i].ry;
          double g_jR[7] = {0.0,-0.08,0.0,0.0,0.0,0.0,0.0};
          double g_jL[7] = {0.0,-0.08,0.0,0.0,0.0,0.0,0.0};
          double g_gripper_right = 0.3;
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

      if (id==2007) // wheelchair
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
            state_data.name.push_back("wheelchair_x_joint");
            state_data.name.push_back("wheelchair_y_joint");
            state_data.name.push_back("wheelchair_yaw_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(rotY);
          }
        }
      }

      // if (id==2009) // refrigerator
      // {
      //   if (state!=0)
      //   {
      //     posX = msg->tmsdb[i].x;
      //     posY = msg->tmsdb[i].y;
      //     rotY = msg->tmsdb[i].ry;
      //
      //     if(posX == 0.0 && posY == 0.0)
      //     {
      //       continue;
      //     }
      //     else
      //     {
      //       state_data.header.stamp = ros::Time::now();
      //       state_data.name.push_back("refrigerator_x_joint");
      //       state_data.name.push_back("refrigerator_y_joint");
      //       state_data.name.push_back("refrigerator_yaw_joint");
      //       state_data.position.push_back(posX);
      //       state_data.position.push_back(posY);
      //       state_data.position.push_back(rotY);
      //     }
      //   }
      // }

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

      if(id==1002) //moverio
      {
        if(state==1){
          posX = msg->tmsdb[i].x;
          posY = msg->tmsdb[i].y;
          rotR = msg->tmsdb[i].rr;
          rotP = msg->tmsdb[i].rp;
          rotY = msg->tmsdb[i].ry;

          if(posX == 0.0 && posY == 0.0)
          {
            continue;
          }
          else if(posX > 9.300 && posX < 11.000 && posY > 2.400 && posY < 3.200) // in the bed
          {
            transform.setOrigin(tf::Vector3(10.52,2.71,0.42));
            transform.setRotation(tf::Quaternion(1.5708,0,1.5708));
            br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world_link","Body"));
          }
          else
          {
            transform.setOrigin(tf::Vector3(posX,posY,1.1));
            transform.setRotation(tf::Quaternion(0,0,-1.5708+rotY));
            br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world_link","Body"));
          }
        }else{

        }
      }

      // if (id==1001) //person
      // {
      //   if (state==1)
      //   {
      //     posX = msg->tmsdb[i].x;
      //     posY = msg->tmsdb[i].y;
      //     rotY = msg->tmsdb[i].ry;
      //
      //     if(posX == 0.0 && posY == 0.0)
      //     {
      //       continue;
      //     }
      //     else
      //     {
      //       // tms_msg_ss::SkeletonArray skeletons;
      //       // skeletons.data.resize(1);
      //       // skeletons.data[0].user_id = 1001;
      //       // skeletons.data[0].position.resize(25);
      //       // skeletons.data[0].orientation.resize(25);
      //       // skeletons.data[0].confidence.resize(25);
      //       // for(int i=0; i<25; i++)
      //       // {
      //       //   skeletons.data[0].position[i].x = 0.0;
      //       //   skeletons.data[0].position[i].y = 0.0;
      //       //   skeletons.data[0].position[i].z = 0.0;
      //       //   skeletons.data[0].orientation[i].w = 1.0;
      //       //   skeletons.data[0].orientation[i].x = 0.0;
      //       //   skeletons.data[0].orientation[i].y = 0.0;
      //       //   skeletons.data[0].orientation[i].z = 0.0;
      //       //   skeletons.data[0].confidence[i] = 0;
      //       // }
      //       // skeletons.data[0].position[0].x = posX;
      //       // skeletons.data[0].position[0].y = posY;
      //       // skeleton_pub.publish(skeletons);
      //     }
      //   }
      // }

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
      if(id==7004) //greentea_bottle
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
            state_data.name.push_back("greentea_bottle_x_joint");
            state_data.name.push_back("greentea_bottle_y_joint");
            state_data.name.push_back("greentea_bottle_z_joint");
            state_data.name.push_back("greentea_bottle_yaw_joint");
            state_data.name.push_back("greentea_bottle_pitch_joint");
            state_data.name.push_back("greentea_bottle_roll_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(posZ);
            state_data.position.push_back(rotY);
            state_data.position.push_back(rotP);
            state_data.position.push_back(rotR);
          }
        }
      }
      if(id==7006) //cancoffee
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
            state_data.name.push_back("cancoffee_x_joint");
            state_data.name.push_back("cancoffee_y_joint");
            state_data.name.push_back("cancoffee_z_joint");
            state_data.name.push_back("cancoffee_yaw_joint");
            state_data.name.push_back("cancoffee_pitch_joint");
            state_data.name.push_back("cancoffee_roll_joint");
            state_data.position.push_back(posX);
            state_data.position.push_back(posY);
            state_data.position.push_back(posZ);
            state_data.position.push_back(rotY);
            state_data.position.push_back(rotP);
            state_data.position.push_back(rotR);
          }
        }
      }
      if(id==7009) //soysauce_bottle_black
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
            state_data.name.push_back("soysauce_bottle_black_x_joint");
            state_data.name.push_back("soysauce_bottle_black_y_joint");
            state_data.name.push_back("soysauce_bottle_black_z_joint");
            state_data.name.push_back("soysauce_bottle_black_yaw_joint");
            state_data.name.push_back("soysauce_bottle_black_pitch_joint");
            state_data.name.push_back("soysauce_bottle_black_roll_joint");
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
