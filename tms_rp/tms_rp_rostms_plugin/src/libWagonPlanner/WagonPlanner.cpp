#include "WagonPlanner.h"
#include <cnoid/MessageView>

using namespace std;
using namespace cnoid;
using namespace grasp;

Wagon::Wagon(cnoid::BodyItemPtr bodyItem){
	slaveItem = bodyItem;
	wagon = slaveItem->body()->link(0);
	master = wagon;
	relativePos = master->R().transpose()*(bodyItem->body()->link(0)->p() - master->p());
	relativeRot = master->R().transpose()*bodyItem->body()->link(0)->R();
	slaveItems.clear();
	state = 1;
	
	rel_control_point.resize(3);
	graspPos_offset.resize(3);
}

WagonPlanner::WagonPlanner()  : 	os (MessageView::mainInstance()->cout() )
{
	targetWagon = NULL;
}

WagonPlanner::~WagonPlanner() {

}

WagonPlanner* WagonPlanner::instance(WagonPlanner *gc) {
	static WagonPlanner* instance = (gc) ? gc : new WagonPlanner();
	if(gc) instance = gc;
	return instance;
}

void WagonPlanner::SetWagon(cnoid::BodyItemPtr bodyItem){

	targetWagon = new Wagon(bodyItem);
	
	targetWagon->rel_th = 0.0;
	targetWagon->state = InterObject::COLLISION;
	targetWagon->wagon_type = Wagon::COLLISION;
	
	changeWagonType(targetWagon, Wagon::COLLISION);
	//~ changeWagonType(targetWagon, Wagon::PUSHING);
	
	PlanBase::instance()->interObjectList.push_back(*targetWagon);
}

void WagonPlanner::SetWagonSlaveItem(cnoid::BodyItemPtr bodyItem){
	InterObject* targetInterObject = new InterObject();
	targetInterObject->slaveItem = bodyItem;
	targetInterObject->master = wagon();
	targetInterObject->relativePos = targetInterObject->master->R().transpose()*(bodyItem->body()->link(0)->p() - targetInterObject->master->p());
	targetInterObject->relativeRot = targetInterObject->master->R().transpose()*bodyItem->body()->link(0)->R();
	
	changeType(targetInterObject, InterObject::COLLISION);
	changeMaster(targetInterObject, wagon());
	targetWagon->slaveItems.push_back(*targetInterObject);
	PlanBase::instance()->interObjectList.push_back(*targetInterObject);
}

void WagonPlanner::RemoveWagonSlaveItem(cnoid::BodyItemPtr bodyItem){
	for(int i=0;i<PlanBase::instance()->interObjectList.size();i++){
		if(bodyItem->body()->name()==PlanBase::instance()->interObjectList[i].slaveItem->name()){
			changeMaster(&(PlanBase::instance()->interObjectList[i]), bodyItem->body()->link(0));
		}
	}
	
}

void WagonPlanner::changeMaster(InterObject* targetInterObject, Link* master){
	targetInterObject->master = master;
	targetInterObject->relativePos = targetInterObject->master->R().transpose()*(targetInterObject->slaveItem->body()->link(0)->p() - targetInterObject->master->p());
	targetInterObject->relativeRot = targetInterObject->master->R().transpose()*targetInterObject->slaveItem->body()->link(0)->R();
	
	for(int i=0;i<PlanBase::instance()->interObjectList.size();i++){
		if(targetInterObject->slaveItem->name()==PlanBase::instance()->interObjectList[i].slaveItem->name()){
			PlanBase::instance()->interObjectList[i].master = targetInterObject->master;
			PlanBase::instance()->interObjectList[i].relativePos = targetInterObject->relativePos;
			PlanBase::instance()->interObjectList[i].relativeRot = targetInterObject->relativeRot;
		}
	}
}

void WagonPlanner::changeType(InterObject* targetInterObject, InterObject::InterObjectType type){
	if(targetInterObject->type == type)
		return;
	
	targetInterObject->type = type;
	
	for(int i=0;i<PlanBase::instance()->interObjectList.size();i++){
		if(targetInterObject->slaveItem->name()==PlanBase::instance()->interObjectList[i].slaveItem->name()){
			PlanBase::instance()->interObjectList[i].type = type;
		}
	}
}

void WagonPlanner::changeObjectType(TargetObject* targetObject, ObjectType obj_type){
	InterObject* targetInterObject = NULL;
	
	for(int i=0;i<PlanBase::instance()->interObjectList.size();i++){
		if(targetObject->name()==PlanBase::instance()->interObjectList[i].slaveItem->name()){
			targetInterObject = &(PlanBase::instance()->interObjectList[i]);
		}
	}
	
	if(targetInterObject==NULL)
		return;
	
	enum ObjectType object_type = ObjectType(obj_type);
	switch(object_type){
		case NOT_GRASPING:
			changeType(targetInterObject, InterObject::COLLISION);
			if(targetInterObject->master != targetInterObject->slaveItem->body()->link(0))
				changeMaster(targetInterObject, targetInterObject->slaveItem->body()->link(0));
			break;
		
		case UNDER_GRASPING:
			break;
		
		case GRASPING:
			changeType(targetInterObject, InterObject::GRASPED_OBJECT);
			if(targetInterObject->master != PlanBase::instance()->palm())
				changeMaster(targetInterObject, PlanBase::instance()->palm());
			break;
		
		case ON_WAGON:
			changeType(targetInterObject, InterObject::GRASPED_OBJECT);
			if(targetInterObject->master != wagon())
				changeMaster(targetInterObject, wagon());
			break;
	}
	
	
}

void WagonPlanner::changeObjectMaster(TargetObject* targetObject, Link* master){
	InterObject* targetInterObject = NULL;
	
	for(int i=0;i<PlanBase::instance()->interObjectList.size();i++){
		if(targetObject->name()==PlanBase::instance()->interObjectList[i].slaveItem->name()){
			targetInterObject = &(PlanBase::instance()->interObjectList[i]);
		}
	}
	
	if(targetInterObject==NULL)
		return;
	
	else
		changeMaster(targetInterObject, master);
}

void WagonPlanner::changeWagonType(Wagon* targetWagon, Wagon::WagonType type){
	if(targetWagon->wagon_type == type)
		return;
	
	InterObject* targetInterObject = NULL;
	
	for(int i=0;i<PlanBase::instance()->interObjectList.size();i++){
		if(targetWagon->slaveItem->name()==PlanBase::instance()->interObjectList[i].slaveItem->name()){
			targetInterObject = &(PlanBase::instance()->interObjectList[i]);
		}
	}
	
	targetWagon->wagon_type = type;
	
	enum Wagon::WagonType wt = Wagon::WagonType(type);
	switch(wt){
		case Wagon::COLLISION:
			changeType(targetInterObject, InterObject::COLLISION);
			if(targetInterObject->master != targetInterObject->slaveItem->body()->link(0))
				changeMaster(targetInterObject, targetInterObject->slaveItem->body()->link(0));
			break;
		
		case Wagon::PUSHING:
			changeType(targetInterObject, InterObject::GRASPED_OBJECT);
			if(targetInterObject->master != PlanBase::instance()->palm())
				changeMaster(targetInterObject, PlanBase::instance()->palm());
			break;
	}
	
	
}

void WagonPlanner::changeWagonMaster(Wagon* targetWagon, Link* master){
	InterObject* targetInterObject = NULL;
	
	for(int i=0;i<PlanBase::instance()->interObjectList.size();i++){
		if(targetWagon->name()==PlanBase::instance()->interObjectList[i].slaveItem->name()){
			targetInterObject = &(PlanBase::instance()->interObjectList[i]);
		}
	}
	
	if(targetInterObject==NULL)
		return;
	
	else
		changeMaster(targetInterObject, master);
}

void WagonPlanner::initialCollision(){
	PlanBase::instance()->initial();
	
	if(targetWagon){
		robotWagonPairs.clear();
		objWagonPairs.clear();
		targetWagon->slaveEnvPairs.clear();
		
		for(unsigned int j=0;j<PlanBase::instance()->bodyItemRobot()->body()->numLinks();j++){
			robotWagonPairs.push_back(make_shared<ColdetLinkPair>(PlanBase::instance()->bodyItemRobot()->body(),PlanBase::instance()->bodyItemRobot()->body()->link(j), targetWagon->wagonItem()->body(), wagon() ));
		}
		ColdetModelPtr backup = wagon()->coldetModel();
		
		if(PlanBase::instance()->targetObject){
			objWagonPairs.push_back(make_shared<ColdetLinkPair>(PlanBase::instance()->targetObject->bodyItemObject->body(), PlanBase::instance()->object(), targetWagon->wagonItem()->body(), wagon() ));
		}
	
		for(unsigned int j=0;j<targetWagon->slaveItem->body()->numLinks();j++){
			for( list<BodyItemPtr>::iterator it = PlanBase::instance()->bodyItemEnv.begin(); it !=PlanBase::instance()->bodyItemEnv.end(); it++){
				for(unsigned int i=0;i<(*it)->body()->numLinks();i++){

					ColdetLinkPairPtr temp= make_shared<ColdetLinkPair>(targetWagon->slaveItem->body(),targetWagon->slaveItem->body()->link(j), (*it)->body(), (*it)->body()->link(i));

					temp->updatePositions();
					int t1,t2;
					double p1[3],p2[3];
					double distance = temp->computeDistance(t1,p1,t2,p2);
					if(distance>1.0e-04)	targetWagon->slaveEnvPairs.push_back(temp);
				#ifdef DEBUG_MODE
					else os <<"collide on initial condition robot and env"  <<distance <<" "<< temp->model(0)->name() <<" " << (*it)->body()->name() << endl;
				#endif
				}
			}
		}
	}
	
	PlanBase::instance()->graspMotionSeq.clear();
}

bool WagonPlanner::isColliding(){
	if(PlanBase::instance()->isColliding())
		return true;
	
	if(targetWagon){
		for(unsigned int i=0;i<targetWagon->slaveEnvPairs.size();i++){
			ColdetLinkPairPtr testPair = targetWagon->slaveEnvPairs[i];
			testPair->updatePositions();
			bool coll = testPair->checkCollision();
			if(coll){
				//colPairName[0] = testPair->model(0)->name();
				//colPairName[1] = testPair->model(1)->name();
		#ifdef DEBUG_MODE
				cout <<"self collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
		#endif
				return true;
			}
		}
		
		if(PlanBase::instance()->targetObject){
			for(unsigned int i=0;i<objWagonPairs.size();i++){
				ColdetLinkPairPtr testPair = objWagonPairs[i];
				testPair->updatePositions();
				bool coll = testPair->checkCollision();
				if(coll){
			#ifdef DEBUG_MODE
					cout <<"self collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
			#endif
					return true;
				}
			}
		}
		
		enum InterObject::InterObjectType inter_object_type = InterObject::InterObjectType(targetWagon->type);
		if(inter_object_type==InterObject::GRASPED_OBJECT){
			for(unsigned int i=0;i<robotWagonPairs.size();i++){
				ColdetLinkPairPtr testPair = robotWagonPairs[i];
				testPair->updatePositions();
				if((testPair->model(0)->name()==targetWagon->master->name())||(testPair->model(1)->name()==targetWagon->master->name()))
					continue;
				bool coll = testPair->checkCollision();
				if(coll){
					//~ //colPairName[0] = testPair->model(0)->name();
					//~ //colPairName[1] = testPair->model(1)->name();
			#ifdef DEBUG_MODE
					cout <<"self collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
			#endif
					return true;
				}
			}
		}
		if(inter_object_type==InterObject::COLLISION){
			for(unsigned int i=0;i<robotWagonPairs.size();i++){
				ColdetLinkPairPtr testPair = robotWagonPairs[i];
				testPair->updatePositions();
				bool coll = testPair->checkCollision();
				if(coll){
					//~ //colPairName[0] = testPair->model(0)->name();
					//~ //colPairName[1] = testPair->model(1)->name();
			#ifdef DEBUG_MODE
					cout <<"self collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
			#endif
					return true;
				}
			}
		}
	}
	return false;
}

WagonMotionState WagonPlanner::getMotionState(double time){
	WagonMotionState w_ret;
	MotionState ret;
	PlanBase* tc = PlanBase::instance();
	ret = tc->getMotionState();
	
	w_ret.jointSeq = ret.jointSeq;
	//~ for(int i=0;i<tc->body()->numJoints();i++)
		//~ w_ret.jointSeq[i] = ret.jointSeq[i];

	w_ret.pos = ret.pos;
	w_ret.rpy = ret.rpy;

	w_ret.graspingState = ret.graspingState;
	w_ret.graspingState2 = ret.graspingState2;
	if( tc->getGraspingState()==PlanBase::GRASPING ){
		w_ret.objectPalmPos = ret.objectPalmPos;
		w_ret.objectPalmRot = ret.objectPalmRot;
	}
	w_ret.objectContactState = ret.objectContactState;
	w_ret.pathPlanDOF = ret.pathPlanDOF;
	w_ret.tolerance = ret.tolerance;
	w_ret.time = ret.time;
	w_ret.id = ret.id;
	
	w_ret.interObjectList = tc->interObjectList;
	w_ret.interObjectList_P.resize(tc->interObjectList.size());
	w_ret.interObjectList_R.resize(tc->interObjectList.size());
	w_ret.interObjectList_type.resize(tc->interObjectList.size());
	for(unsigned int i=0;i<w_ret.interObjectList.size();i++){
		w_ret.interObjectList_P[i] = tc->interObjectList[i].slaveItem->body()->link(0)->p();
		w_ret.interObjectList_R[i] = tc->interObjectList[i].slaveItem->body()->link(0)->R();
		w_ret.interObjectList_type[i] = InterObject::InterObjectType(tc->interObjectList[i].type);
	}
	
	if(targetWagon){
		w_ret.wagon_type = Wagon::WagonType(targetWagon->wagon_type);
	}
	
	return w_ret;
}

void WagonPlanner::setMotionState(WagonMotionState gm){
	PlanBase* tc = PlanBase::instance();
	
	for(int i=0;i<tc->body()->numJoints();i++){
		tc->body()->joint(i)->q() = gm.jointSeq[i];
	}
	tc->body()->link(0)->p() = gm.pos;
	tc->body()->link(0)->R() = rotFromRpy(gm.rpy);

	tc->setGraspingState(gm.graspingState);
	tc->setGraspingState2(gm.graspingState2);
	if( tc->getGraspingState()==PlanBase::GRASPING ){
		tc->targetArmFinger->objectPalmPos = gm.objectPalmPos;
		tc->targetArmFinger->objectPalmRot = gm.objectPalmRot;
	}
	tc->setObjectContactState(gm.objectContactState);
	tc->pathPlanDOF = gm.pathPlanDOF;
	tc->setTolerance(gm.tolerance);
	tc->calcForwardKinematics();
	//~ tc->motionId = gm.id;
	
	tc->interObjectList = gm.interObjectList;
	for(unsigned int i=0;i<tc->interObjectList.size();i++){
		tc->interObjectList[i].slaveItem->body()->link(0)->p() = gm.interObjectList_P[i];
		tc->interObjectList[i].slaveItem->body()->link(0)->R() = gm.interObjectList_R[i];
		tc->interObjectList[i].type = gm.interObjectList_type[i];
		
		tc->interObjectList[i].slaveItem->body()->calcForwardKinematics();
		tc->interObjectList[i].slaveItem->notifyKinematicStateChange();
	}
	
	if(targetWagon)
		targetWagon->wagon_type = gm.wagon_type;
}
