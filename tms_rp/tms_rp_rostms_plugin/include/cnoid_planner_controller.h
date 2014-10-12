#ifndef _CNOID_PLANNER_CONTROLLER_H_INCLUDED
#define _CNOID_PLANNER_CONTROLLER_H_INCLUDED

#include <Grasp/PlanBase.h>
#include <Grasp/GraspController.h>
#include <libWagonPlanner/WagonTrajectoryPlanner.h>
#include <libWagonPlanner/WagonPushPlanner.h>
#include <libGiveObjToHumanPlanner/HumanPlanner.h>
#include <ros/ros.h>
#include <cnoid/MessageView>
#include <Grasp/exportdef.h>

#define rad2deg(x)	((x)*(180.0)/M_PI)
#define deg2rad(x)	((x)*M_PI/180.0)

using namespace cnoid;

namespace grasp {
	
class RPS_Controller{
	
	public:
		RPS_Controller();
		static RPS_Controller* instance(RPS_Controller *gc=NULL);
		virtual ~RPS_Controller();
		
		bool SetRobotPose(vector<double> pos, vector<double> rpy, vector<double> joint_angle);
		bool SetWagonPose(vector<double> pos, vector<double> rpy);
		bool SetObjectPose(vector<double> pos, vector<double> rpy);
		bool SetHumanPose(vector<double> pos, vector<double> rpy, vector<double> joint_angle);
		
		bool GraspObjPlan(vector<vector<double> >& out_joint_angle);
		bool calcGraspWagonPose(vector<double>& out_joint_angle);
		bool GraspWagonPlan(int& grasp_type, vector<vector<double> >& out_joint_angle, double& out_palm_th, vector<double>& rel_control_point_pos, double& rel_th);
		bool ReleaseWagonPlan(vector<vector<double> >& out_joint_angle);
		
	//~ private:
		std::ostream& os;
		
		PlanBase* pb;
		WagonPlanner* wp;
		WagonPushPlanner* wpp;
		HumanPlanner* hp;
  
};
}

#endif
