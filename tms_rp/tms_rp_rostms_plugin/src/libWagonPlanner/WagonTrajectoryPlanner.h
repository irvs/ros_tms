#ifndef _WagonTrajectoryPlanner_H
#define _WagonTrajectoryPlanner_H

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <../src/PoseSeqPlugin/PoseSeqItem.h>  
#include <cnoid/BodyMotion>  	/* modified by qtconv.rb 0th rule*/  

#include <cnoid/EigenTypes>	/* modified by qtconv.rb 0th rule*/  


#include "./extplugin/graspPlugin/Grasp/PlanBase.h"
#include "./extplugin/graspPlugin/PRM/TrajectoryPlanner.h"
#include "WagonPlanner.h"

#include "exportdef.h"

namespace grasp{


class EXCADE_API WagonTrajectoryPlanner : public grasp::TrajectoryPlanner {

	public :

	WagonTrajectoryPlanner(int id=0);
	virtual ~WagonTrajectoryPlanner(){};
		
	bool doWagonTrajectoryPlanning();
		
	bool updateTrajectoryFromMotion(const cnoid::BodyMotionPtr motionObject, const cnoid::BodyMotionPtr motionRobot, std::vector<WagonMotionState>& wagon_motionSeq);
	bool updateWagonTrajectoryFromMotion(int InterObjectId, std::vector<InterObject> interObjectList, const cnoid::BodyMotionPtr motionObject, const cnoid::BodyMotionPtr motionRobot, std::vector<WagonMotionState>& motionSeq);
	
	vector<cnoid::PoseSeqItem*> poseSeqInterObject;
	
	std::vector<WagonMotionState> wagon_motionSeq;

};


}


#endif
