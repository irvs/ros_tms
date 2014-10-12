#include <cnoid/Plugin>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ItemManager>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/BodyMotionItem>	/* modified by qtconv.rb 0th rule*/  

//#include <rtm/Manager.h>
#include <iostream>
#include <string>
#include <stdlib.h>

#include "WagonBar.h"
#include "WagonTrajectoryBar.h"

using namespace cnoid;
using namespace grasp;


namespace {
    class WagonPlannerPlugin : public Plugin
    {
    public:
        
        WagonPlannerPlugin() : Plugin("WagonPlanner") {
            depend("Grasp");
        }

        bool initialize() {
	
			addToolBar(grasp::WagonBar::instance());
			addToolBar(grasp::WagonTrajectoryBar::instance());
            
            return true;
        }
    };
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(WagonPlannerPlugin);
