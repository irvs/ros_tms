#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/BodyMotionItem>

#include <iostream>
#include <string>
#include <stdlib.h>

#include "GiveObjToHumanPlannerBar.h"

using namespace cnoid;
using namespace grasp;

namespace {
    class GiveObjToHumanPlannerPlugin : public Plugin
    {
    public:
        
        GiveObjToHumanPlannerPlugin() : Plugin("GiveObjToHumanPlanner") {
            depend("Grasp");
        }

        bool initialize() {
	
			addToolBar(grasp::GiveObjToHumanPlannerBar::instance());
			//~ addToolBar(grasp::WagonTrajectoryBar::instance());
            
            return true;
        }
    };
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(GiveObjToHumanPlannerPlugin);
