#ifndef  EXAMPLE_BAR_H
#define  EXAMPLE_BAR_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tms_rp_controller.h>

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

namespace grasp 
{

class ExampleBar : public cnoid::ToolBar, public boost::signals::trackable 
{
 public:
  int argc;
  char **argv;

  ExampleBar();
  static ExampleBar* instance();
  virtual ~ExampleBar();

 private:

  static bool isRosInit;

  MessageView& mes;
  std::ostream& os;
  TmsRpController& trc_;

  void onTestButtonClicked();
  void rosOn();
};

}

#endif //EXAMPLE_PLUGIN_BAR_H
