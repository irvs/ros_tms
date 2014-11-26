#ifndef  TMS_RP_VIEWER_BAR_H
#define  TMS_RP_VIEWER_BAR_H

#include <tms_rp_controller.h>
//#include <tms_rp_bar.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <tms_msg_db/TmsdbStamped.h>

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
  tms_msg_db::TmsdbStamped environment_information_;
  ros::Subscriber subscribe_environment_information_;

private:
  static bool isRosInit;

  MessageView& mes;
  std::ostream& os;
  grasp::TmsRpController& trc_;
//  grasp::TmsRpBar& trb_;

  void updateEnvironmentInformation(const tms_msg_db::TmsdbStamped::ConstPtr& msg);

  void onViewerClicked();
  void rosOn();
};

}

#endif //TMS_RP_VIEWER_BAR_H
