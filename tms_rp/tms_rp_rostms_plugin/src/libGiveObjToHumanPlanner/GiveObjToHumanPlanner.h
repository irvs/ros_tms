#ifndef _GIVEOBJTOHUMAN_PLANNER_H
#define _GIVEOBJTOHUMAN_PLANNER_H

#include "HumanPlanner.h"

#include "./extplugin/graspPlugin/Grasp/PlanBase.h"
#include "./extplugin/graspPlugin/Grasp/GraspController.h"
#include "./extplugin/graspPlugin/Grasp/VectorMath.h"
#include "../libWagonPlanner/WagonPlanner.h"
#include "exportdef.h"

using namespace cnoid;
using namespace std;

#define rad2deg(x)	((x)*(180.0)/M_PI)
#define deg2rad(x)	((x)*M_PI/180.0)

struct manip_map_data{
	bool reachable;
	
	double robot_manipulability;
	double human_manipulability;
	double peak_manipulability;
	double total_manipulability;
	
	Vector3 obj_pos;
	Vector3 robot_palm_pos_rel;
	
	double dist_from_obj_to_robot;
	double th_from_obj_to_robot;
	
	vector<double>	human_joint_angle;
	vector<double>	robot_joint_angle;
};

namespace grasp{

	class EXCADE_API GiveObjToHumanPlanner
	{
		public:
			GiveObjToHumanPlanner();
			~GiveObjToHumanPlanner();
			
			static GiveObjToHumanPlanner* instance(GiveObjToHumanPlanner *gc=NULL);
			
			ostream& os;
			
			vector<manip_map_data> manip_MAP, robot_manip_MAP;
			
			void makeManipulabilityMap(char model_type, bool useWaist);	//model_type: 'R'=robot, 'H'=human
			bool SetRobotManipulabilityMap(char robot_LR, bool robot_useWaist);
			bool unifyManipulabilityMap(Vector3 human_pos, double human_th, int human_height, char robot_LR, bool robot_useWaist, char human_LR, bool human_useWaist);
			void QSort_manip_data_total(vector<manip_map_data>& m_Map, int left, int right);	//sort by total_manipulability
			void QSort_manip_data_peak(vector<manip_map_data>& m_Map, int left, int right);	//sort by peak_manipulability
			
			void calcRobotPos_GiveObj(unsigned int rank, vector<vector<double> > &out_posList);
			void calcRobotPos_GetObj(unsigned int rank, vector<vector<double> > &out_posList);
			void expandRobotObjDist(Vector3 objPos, double expand_dist, vector<vector<double> > &out_posList);
			void dividePosList(unsigned int k, vector<vector<double> > posList, vector<vector<vector<double> > >& out_posList);
			void removeCollisionPos(vector<vector<double> > in_posList, vector<vector<double> >& out_posList);
			//~ void QSort_posList_distance(Vector3 target_pos, vector<vector<double> >& posList, int left, int right);
			void QSort_posList_distance(vector<vector<double> >& posList, int left, int right);
			void calc_posList_distance(Vector3 target_pos, vector<vector<double> >& posList);
	};
	
};
#endif
