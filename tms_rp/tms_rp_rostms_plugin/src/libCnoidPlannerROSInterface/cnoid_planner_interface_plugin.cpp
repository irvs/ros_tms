#include <cnoid_planner_interface_bar.h>
//~ #include <tms_rp_controller.h>

#include <ros/ros.h>
#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/BodyMotionItem>
#include <Grasp/PlanBase.h>

#include <iostream>
#include <string>
#include <stdlib.h>

using namespace cnoid;
using namespace grasp;

namespace {
    class PlannerROSInterfacePlugin : public Plugin
    {
    public:
        
        PlannerROSInterfacePlugin() : Plugin("PlannerROSInterface") {
            depend("Grasp");
        }

        bool initialize() {
                addToolBar(grasp::PlannerROSInterfaceBar::instance());
		return true;
        }
    };
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(PlannerROSInterfacePlugin);
