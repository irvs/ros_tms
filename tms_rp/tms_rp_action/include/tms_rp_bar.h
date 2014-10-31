#ifndef _TMS_ACTION_BAR_H_INCLUDED
#define _TMS_ACTION_BAR_H_INCLUDED

#include <tms_rp_controller.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tms_msg_rp/rps_path_planning.h>
#include <tms_msg_rc/rc_robot_control.h>
#include <tms_msg_rc/robot_control.h>
#include <tms_msg_db/TmsdbGetData.h>
#include <tms_msg_db/Tmsdb.h>
#include <tms_msg_db/TmsdbStamped.h>

#include <tms_msg_rp/rps_position.h>
#include <tms_msg_rp/rps_route.h>
#include <tms_msg_rp/rps_map_data.h>
#include <tms_msg_rp/rps_map_y.h>
#include <tms_msg_rp/rps_map_full.h>

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
#include <Grasp/exportdef.h>
#include <PRM/TrajectoryPlanner.h>

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

#define PERSON 1
//#define PERSON 0

using namespace std;
using namespace cnoid;

namespace grasp {
class SelectGoalPosDialog : public QDialog
{
  public:
    cnoid::SpinBox goal_pos_x;
    cnoid::SpinBox goal_pos_y;
    cnoid::SpinBox goal_pos_ry;
    SelectGoalPosDialog() ;
    void okClicked();
};

class TmsRpBar : public cnoid::ToolBar, public boost::signals::trackable {
 public:
  TmsRpBar();
  static TmsRpBar* instance();
  virtual ~TmsRpBar();

  void onMoveToGoal();
  void onSimulationInfoButtonClicked();

  int argc;
  char **argv;
  uint32_t sid;
  ros::ServiceClient get_data_client;
  ros::ServiceClient sp5_control_client;
  ros::ServiceClient path_planning_client;
  ros::ServiceClient ardrone_client;
  ros::Subscriber    subscribe_pcd;
  ros::Subscriber    subscribe_map;

  pcl::PointCloud<pcl::PointXYZ> pointCloudData;
  tms_msg_rp::rps_map_full       staticMapData;

  boost::signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)>& sigBodyItemSelectionChanged() {return sigBodyItemSelectionChanged_;}

  bool objectState[25];
  static std::string objectName[25];
  static std::string furnitureName[20];
  static bool isRosInit;

  double goal_position_x;
  double goal_position_y;
  double goal_position_ry;

  static bool production_version;
  static int planning_mode; // 0:view mode / 1:planning mode
  static int grasping;

 private:
  MessageView& mes;
  std::ostream& os;
  TmsRpController& tac;
  Matrix3d mat0, mat_ccw90, mat_ccw180, mat_cw90;

  cnoid::BodyItemPtr currentBodyItem_;
  cnoid::ItemList<cnoid::BodyItem> selectedBodyItems_;
  cnoid::ItemList<cnoid::BodyItem> targetBodyItems;

  boost::signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;

  void onItemSelectionChanged(const cnoid::ItemList<cnoid::BodyItem>& bodyItems);
  void StaticMapButtonClicked();
  void receivePointCloudData(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void receiveMapData(const tms_msg_rp::rps_map_full::ConstPtr& msg);
  void getPcdData();

  void UpdateObjectInfo();
  void onInitPoseButtonClicked();
  void onStartButtonClicked();
  void onStartButtonClicked2();
  void onChangePlanningMode();
  void onPathPlanButtonClicked();
  void simulationButtonClicked();
  void ConnectRosButtonClicked();
  void simulation();
  void ConnectROS();
  void ardroneButtonClicked();
};
}

#endif
