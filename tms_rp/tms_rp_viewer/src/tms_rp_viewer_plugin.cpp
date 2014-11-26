#include <tms_rp_viewer_bar.h>

#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/BodyMotionItem>

#include <iostream>
#include <string>
#include <stdlib.h>

using namespace cnoid;
using namespace grasp;

namespace 
{

class RpViewerPlugin : public Plugin
{
  public:

  RpViewerPlugin() : Plugin("Example")
  {
  }

  bool initialize()
  {
    addToolBar(tms_rp::RpViewerBar::instance());
    return true;
  }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(RpViewerPlugin);
