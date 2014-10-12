#ifndef _WAGON_PUSH_PLANNER_H
#define _WAGON_PUSH_PLANNER_H

#include "WagonPlanner.h"
#include "../libVoronoiPathPlanner/VoronoiPathPlanner.h"
#include "../libGiveObjToHumanPlanner/GiveObjToHumanPlanner.h"

#define rad2deg(x)	((x)*(180.0)/M_PI)
#define deg2rad(x)	((x)*M_PI/180.0)

using namespace cnoid;
using namespace std;

namespace grasp{
	
	class EXCADE_API WagonPushPlanner
	{
		public:
			WagonPushPlanner();
			~WagonPushPlanner();
			
			static WagonPushPlanner* instance(WagonPushPlanner *gc=NULL);
			
			ostream& os;
			
			//----Robot----//
			ArmFingers* robot;
			Link* rl;
			double rth;
			
			//----Wagon-----//
			Wagon* wagon;
			Link* wl;
			double wth;
			
			void setStandardRobotPose();
			void setRobotPose(vector<double> pose);
			void calcWagonPolePos(vector<vector<double> >& out_pos);
			void calcWagonGraspBasePos(double distance, vector<vector<double> >& out_pos);
			bool calcWagonGraspPose(vector<double> robot_pos, double& out_palm_th, vector<double>& out_rel_control_pos, double& out_rel_th, vector<double>& out_pose);
			bool calcStandardWagonGraspPose(vector<double> robot_pos, int grasp_type, vector<double>& out_pose);
			bool calcRelativeWagonGraspPose(vector<double> robot_pos, int grasp_type, vector<double> rel_control_pos, double rel_th, vector<double>& out_pose);
			bool calcPrePose(vector<double> robot_pos, Vector3 palm_pos, double palm_th, char LR, bool use_waist, Vector3 offset, vector<double>& out_pose);
			
			void WagonMoveStandardPosPlan(vector<double> robot_pos, int grasp_type, double rel_th, vector<double>& out_pose);
			void WagonGraspPlan(WagonMotionState preMotionState, vector<WagonMotionState>& outWagonGraspMotionSeq);
			void WagonReleasePlan(WagonMotionState preMotionState, vector<WagonMotionState>& outWagonGraspMotionSeq);
			
			//use voronoi path planner//
			bool calcWagonPosOnVoronoi(vector<vector<CollisionMapData> >& Map, vector<double>& robot_pos, int wagon_state, double control_dist, double& rel_th, bool change_rel, double threshold);
			bool calcWagonPushPath(vector<vector<CollisionMapData> >& Map, vector<double> start, vector<double> goal, vector<vector<double> >& out_path);
	};
	
};
#endif
