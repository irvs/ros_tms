//------------------------------------------------------------------------------
// @file   : umo_tracker.cpp
// @brief  : Tracker of Unidentified Moving Object (UMO)
// @author : Yoonseok Pyo
// @version: Ver0.0.1 (since 2014.11.21)
// @date   : 2014.11.21
//------------------------------------------------------------------------------
#include <ros/ros.h>
#include <tms_msg_ss/tracking_points.h>
#include <tms_msg_db/TmsdbGetData.h>

//------------------------------------------------------------------------------
class UmoTracker
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber tracker_sub_;
  ros::ServiceClient get_data_client_;
  void msgCallback(const tms_msg_ss::tracking_points::ConstPtr& msg)
  {
    unsigned int object_num = msg->tracking_grid.size();
    ROS_INFO("object num: %d", object_num);

    tms_msg_db::TmsdbGetData getRobotData;
    bool isRobot=false;
    float robot_x, robot_y,distance_robot_and_object;
    float obstacle_pos_x = 0, obstacle_pos_y = 0;

    getRobotData.request.tmsdb.id     = 2003; //sp5_2
    getRobotData.request.tmsdb.sensor = 3001; //ID of Vicon sensor

    if (!get_data_client_.call(getRobotData))
    {
      isRobot = false;
    }
    else if (getRobotData.response.tmsdb.empty()==true)
    {
      isRobot = false;
    }
    else
    {
      isRobot = true;
    }

    if (isRobot == true && getRobotData.response.tmsdb[0].state==1)
    {
      robot_x = getRobotData.response.tmsdb[0].x/1000;
      robot_y = getRobotData.response.tmsdb[0].y/1000;
    }

    for(unsigned int i=0;i<object_num;i++)
    {

      obstacle_pos_x = msg->tracking_grid[i].x/1000;
      obstacle_pos_y = msg->tracking_grid[i].y/1000;

      if(isRobot==true)
      {
        distance_robot_and_object = sqrt((obstacle_pos_x-robot_x)*(obstacle_pos_x-robot_x)+(obstacle_pos_y-robot_y)*(obstacle_pos_y-robot_y));
        if(distance_robot_and_object < 1.0)
        {
          continue;
        }
      }
    }
  }

public:
  UmoTracker()
  {
    tracker_sub_ = nh_.subscribe("tracking_points", 10, &UmoTracker::msgCallback, this);
    get_data_client_ = nh_.serviceClient<tms_msg_db::TmsdbGetData>("/tms_db_reader/dbreader");
  }
  ~UmoTracker()
  {
  }
};

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "umo_tracker");
  UmoTracker ut;
  ros::spin();
  return 0;
}

//------------------------------------------------------------------------------
//EOF
