#ifndef _WAGON_PLANNER_H
#define _WAGON_PLANNER_H

#include "./extplugin/graspPlugin/Grasp/InterObject.h"
#include "./extplugin/graspPlugin/Grasp/PlanBase.h"
#include "./extplugin/graspPlugin/Grasp/VectorMath.h"
#include "exportdef.h"
#include <boost/make_shared.hpp>

using namespace boost;
using namespace cnoid;
using namespace std;

namespace grasp{

	class EXCADE_API Wagon : public InterObject
	{
		public:
			Wagon(BodyItemPtr bodyItem);
			const string& name() { return slaveItem->name(); }
			cnoid::BodyItemPtr wagonItem() { return slaveItem; }
			
			vector<double> rel_control_point;
			double rel_th;
			double size_LongSide_Length;
			double size_ShortSide_Length;
			double size_Height;
			double controlDist;
			double graspHeight;
			vector<double> graspPos_offset;

			Link *wagon;
			
			int wagon_type;
			enum WagonType{ COLLISION, PUSHING };
			
			vector<InterObject> slaveItems;
			
	};
	
	class EXCADE_API WagonMotionState : public MotionState
	{
		public:
			vector<InterObject> interObjectList;
			vector<Vector3> interObjectList_P;
			vector<Matrix3> interObjectList_R;
			vector<InterObject::InterObjectType> interObjectList_type;
			Wagon::WagonType	wagon_type;
			enum ObjectType{ NOT_GRASPING, UNDER_GRASPING, GRASPING, ON_WAGON };
			ObjectType	object_type;
	};
	
	class EXCADE_API WagonPlanner
	{
		public:
			WagonPlanner();
			~WagonPlanner();
			
			static WagonPlanner* instance(WagonPlanner *gc=NULL);
			
			void SetWagon(BodyItemPtr bodyItem);
			void SetWagonSlaveItem(BodyItemPtr bodyItem);
			void RemoveWagonSlaveItem(BodyItemPtr bodyItem);

			enum ObjectType{ NOT_GRASPING, UNDER_GRASPING, GRASPING, ON_WAGON };
			void changeMaster(InterObject* targetObject, Link* master);
			void changeType(InterObject* targetInterObject, InterObject::InterObjectType type);
			void changeObjectType(TargetObject* targetObject, ObjectType obj_type);
			void changeObjectMaster(TargetObject* targetObject, Link* master);
			void changeWagonType(Wagon* targetWagon, Wagon::WagonType type);
			void changeWagonMaster(Wagon* targetWagon, Link* master);
			
			void initialCollision();
			bool isColliding();

			WagonMotionState getMotionState(double time=0);
			void setMotionState(WagonMotionState gm);
			
			Wagon* targetWagon;
			InterObject* targetInterObject;
			
			ostream& os;
			
			Link* wagon() { return targetWagon->wagon; }
			
			vector<ColdetLinkPairPtr> robotWagonPairs, objWagonPairs;
			
			// MotionStateForPathPlanner
			WagonMotionState startMotionState;
			WagonMotionState endMotionState;
			WagonMotionState stopWagonMotionState;
			vector<WagonMotionState> wagonMotionSeq;
		
	};
	
};
#endif
