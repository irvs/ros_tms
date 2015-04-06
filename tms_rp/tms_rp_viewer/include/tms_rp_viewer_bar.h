#ifndef  TMS_RP_VIEWER_BAR_H
#define  TMS_RP_VIEWER_BAR_H

#include <tms_rp_controller.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_rp/rps_path_planning.h>
#include <tms_msg_rp/rps_position.h>
#include <tms_msg_rp/rps_route.h>
#include <tms_msg_rp/rps_map_data.h>
#include <tms_msg_rp/rps_map_y.h>
#include <tms_msg_rp/rps_map_full.h>
#include <tms_msg_ss/tracking_points.h>

#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/SpinBox>
#include <cnoid/Button>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <cnoid/SignalProxy>
#include <cnoid/MessageView>
#include <cnoid/MainWindow>
#include <cnoid/LazyCaller>

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <sstream>
#include <fstream>
#include <time.h>
#include <cstdlib>
#include <string>
#include <iostream>

#include <QDialog>
#include <QCheckBox>
#include <QLayout>
#include <QCheckBox>
#include <QPushButton>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#define MAX_ICS_OBJECT_NUM    25
#define MAX_FURNITURE_NUM     24

using namespace std;
using namespace cnoid;

namespace tms_rp
{

class RpViewerBar : public cnoid::ToolBar, public boost::signals::trackable 
{
public:
  RpViewerBar();
  static RpViewerBar* instance();
  virtual ~RpViewerBar();

  int argc;
  char **argv;

  ros::Subscriber subscribe_environment_information_;
  ros::Subscriber subscribe_static_map_;
  ros::Subscriber subscribe_dynamic_map_;
  ros::Subscriber subscribe_path_map_;
  ros::Subscriber subscribe_lrf_raw_data1_;
  ros::Subscriber subscribe_lrf_raw_data2_;
  ros::Subscriber subscribe_pcd_;
  ros::Subscriber subscribe_umo_tracker_;

  tms_msg_db::TmsdbStamped       environment_information_;
  tms_msg_rp::rps_map_full       static_map_data_;
  tms_msg_rp::rps_map_full       dynamic_map_data_;
  tms_msg_rp::rps_route          path_map_data_;
  sensor_msgs::LaserScan         lrf_raw_data1_;
  sensor_msgs::LaserScan         lrf_raw_data2_;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_data_;
  tms_msg_ss::tracking_points    unknown_moving_object_position_;

private:
  static bool isRosInit;
  static std::string object_name_[MAX_ICS_OBJECT_NUM];
  static std::string furniture_name_[MAX_FURNITURE_NUM];
  Matrix3d mat0_, mat_ccw90_, mat_ccw180_, mat_cw90_;
  ToolButton* visibility_toggle_;
  ToolButton* static_map_toggle_;
  ToolButton* dynamic_map_toggle_;
  ToolButton* local_map_toggle_;
  ToolButton* path_map_toggle_;
  ToolButton* robot_map_toggle_;
  ToolButton* point2d_toggle_;
  ToolButton* person_toggle_;

  MessageView& mes;
  std::ostream& os;
  grasp::TmsRpController& trc_;

  void receiveEnvironmentInformation(const tms_msg_db::TmsdbStamped::ConstPtr& msg);
  void receiveStaticMapData(const tms_msg_rp::rps_map_full::ConstPtr& msg);
  void receiveDynamicMapData(const tms_msg_rp::rps_map_full::ConstPtr& msg);
  void receivePathMapData(const tms_msg_rp::rps_route::ConstPtr& msg);
  void receiveLrfRawData1(const sensor_msgs::LaserScan::ConstPtr& msg);
  void receiveLrfRawData2(const sensor_msgs::LaserScan::ConstPtr& msg);
  void receivePointCloudData(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void receiveUnknownMovingObjectTrackerInfo(const tms_msg_ss::tracking_points::ConstPtr& msg);

  void updateEnvironmentInformation(bool is_simulation);

  void viewStaticMap();
  void viewDynamicMap();
  void viewPathOfRobot();
  void viewMarkerOfRobot();
  void viewLrfRawData();
  void viewPersonPostion();
  void viewVitalSensor();

  void onViewerClicked();
  void rosOn();
};

}

#endif //TMS_RP_VIEWER_BAR_H
