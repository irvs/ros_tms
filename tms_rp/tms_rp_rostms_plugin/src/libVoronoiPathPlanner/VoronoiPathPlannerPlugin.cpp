#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/BodyMotionItem>

#include <iostream>
#include <string>
#include <stdlib.h>

#include "VoronoiPathPlannerBar.h"

using namespace cnoid;
using namespace grasp;


namespace {
    class VoronoiPathPlannerPlugin : public Plugin
    {
    public:
        
        VoronoiPathPlannerPlugin() : Plugin("VoronoiPathPlanner") {
            depend("Grasp");
        }

        bool initialize() {
	
			addToolBar(grasp::VoronoiPathPlannerBar::instance());
            
            return true;
        }
    };
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(VoronoiPathPlannerPlugin);
