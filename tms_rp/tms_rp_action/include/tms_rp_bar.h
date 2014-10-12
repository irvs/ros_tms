#ifndef _TMS_ACTION_BAR_H_INCLUDED
#define _TMS_ACTION_BAR_H_INCLUDED

#include <tms_rp_controller.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>

#include <tms_msg_rp/rps_path_planning.h>
#include <tms_msg_rc/rc_robot_control.h>
#include <tms_msg_rc/robot_control.h>
#include <tms_msg_db/TmsdbGetData.h>
#include <tms_msg_db/Tmsdb.h>
#include <tms_msg_db/TmsdbStamped.h>
#include <PRM/TrajectoryPlanner.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/SpinBox>
#include <cnoid/Button>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sstream>
#include <fstream>
#include <time.h>

#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <cnoid/SignalProxy>
#include <cnoid/MessageView>
#include <cnoid/MainWindow>
#include <cnoid/LazyCaller>
#include <Grasp/exportdef.h>

#include <QDialog>
#include <QCheckBox>
#include <QLayout>
#include <QCheckBox>
#include <QPushButton>

#define PERSON 1
//#define PERSON 0

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

  bool objectState[25];
  static std::string objectName[25];
  static std::string furnitureName[16];
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

  void onUpdateInfoButtonClicked();
  void onInitPoseButtonClicked();
  void onStartButtonClicked();
  void onStartButtonClicked2();
  void onChangePlanningMode();
  void onPathPlanButtonClicked();
  void simulationButtonClicked();
  void standbyButtonClicked();
  void simulation();
  void standby();
  void ardroneButtonClicked();
};
}

#endif
