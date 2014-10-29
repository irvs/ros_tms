#include <tms_rp_bar.h>
#include <tms_rp_controller.h>
#include <tms_rp_rp.h>
#include <tms_rp_pp.h>

#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/BodyMotionItem>

#include <iostream>
#include <string>
#include <stdlib.h>

using namespace cnoid;
using namespace grasp;


namespace {
  class TmsRpPlugin : public Plugin
  {
    public:
      TmsRpPlugin() : Plugin("TmsRp") {
        depend("Trajectory");
      }

      bool initialize() {
        addToolBar(grasp::TmsRpBar::instance());
        addToolBar(tms_rp::TmsRpSubtask::instance());
        addToolBar(tms_rp::TmsRpPathPlanning::instance());
        addToolBar(tms_rp::TmsRpView::instance());
        return true;
      }
  };
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(TmsRpPlugin);
