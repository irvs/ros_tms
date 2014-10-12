#ifndef _PLANNER_ROS_INTERFACE_BAR_H_INCLUDED
#define _PLANNER_ROS_INTERFACE_BAR_H_INCLUDED

#include <ros/ros.h>

#include <cnoid/ToolBar>
#include <cnoid/MessageView>
#include <Grasp/exportdef.h>
#include <cnoid_planner_controller.h>

using namespace cnoid;

namespace grasp {
class PlannerROSInterfaceBar : public cnoid::ToolBar, public boost::signals::trackable {
 public:
  PlannerROSInterfaceBar();
  static PlannerROSInterfaceBar* instance();
  virtual ~PlannerROSInterfaceBar();

  bool isRosInit;

 private:
  MessageView& mes;
  std::ostream& os;
  
  void onUpdatePosButtonClicked();
  void onStartRPSServiceServerButtonClicked();
  void onTestButtonClicked();
  
};
}

//~ bool pose_set(skeleton_pose_set::pose_setting::Request &req, skeleton_pose_set::pose_setting::Response &res);

#endif
