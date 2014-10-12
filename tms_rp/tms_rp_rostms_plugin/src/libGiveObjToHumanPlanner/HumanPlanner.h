#ifndef _HUMAN_PLANNER_H
#define _HUMAN_PLANNER_H

#include "./extplugin/graspPlugin/Grasp/PlanBase.h"
#include "./extplugin/graspPlugin/Grasp/VectorMath.h"
#include "../libWagonPlanner/WagonPlanner.h"
#include "exportdef.h"

using namespace cnoid;
using namespace std;

namespace grasp{

	class EXCADE_API Human : public cnoid::Item
	{
		public:
			Human(BodyItemPtr bodyItem, const cnoid::YamlMapping& gSettings);
			const std::string& name() { return bodyItemHuman->name(); }

			BodyItemPtr bodyItemHuman;
			
			int nFing;
			int nHandLink;

			Link *palm;
			Link *base;

			FingerPtr *fingers;
			LinkTraverse  *handJoint;

			ArmPtr arm;
			string pythonInterface;

			multimap<string,string> contactLinks;

	};
	
	class EXCADE_API HumanPlanner
	{
		public:
			HumanPlanner();
			~HumanPlanner();
			
			static HumanPlanner* instance(HumanPlanner *gc=NULL);
			
			void SetHuman(BodyItemPtr bodyItem);
			
			void setHumanPosition(Vector3 pos, Matrix3 rot);
			void setHumanPose(vector<double> angle);

			void initialCollision();
			bool isColliding();
			
			Human* targetHuman;
			vector<Human*> humanList;
			
			BodyPtr body() { return targetHuman->bodyItemHuman->body(); }
			BodyPtr body(int i) { return humanList[i]->bodyItemHuman->body(); }
			Link* palm() { return targetHuman->palm; }
			Link* palm(int i) { return humanList[i]->palm; }
			Link* base() { return targetHuman->base; }
			Link* base(int i) { return humanList[i]->base; }
			ArmPtr arm(){ return targetHuman->arm; }
			ArmPtr arm(int i){ return humanList[i]->arm; }
			FingerPtr fingers(int i) { return targetHuman->fingers[i]; }
			FingerPtr fingers(int i, int j) { return humanList[i]->fingers[j]; }
			
			ostream& os;
			bool flush();
			
			vector<ColdetLinkPairPtr> humanSelfPairs, robotHumanPairs, wagonHumanPairs, objHumanPairs;
			
			Link* human() { return targetHuman->base; }
			
	};
	
};
#endif
