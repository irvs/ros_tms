#include <skeleton_pose_set_bar.h>

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
    class SkeletonPosePlugin : public Plugin
    {
    public:
        
        SkeletonPosePlugin() : Plugin("SkeletonPoseSet") {
            depend("Grasp");
        }

        bool initialize() {
                addToolBar(grasp::SkeletonPoseBar::instance());
		return true;
        }
    };
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(SkeletonPosePlugin);
