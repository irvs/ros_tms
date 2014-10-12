#include "HumanPlanner.h"
#include <cnoid/MessageView>

#include <boost/filesystem.hpp>

using namespace std;
using namespace cnoid;
using namespace grasp;

Human::Human(BodyItemPtr bodyItem, const YamlMapping& gSettings){
	bodyItemHuman = bodyItem;
	
	//READ YAML setting
	palm = bodyItemHuman->body()->link(gSettings["palm"].toString());
	base = bodyItemHuman->body()->link(gSettings["base"].toString());

	const YamlSequence& tips = *gSettings["fingerEnds"].toSequence();

	nFing = tips.size();
	fingers = new FingerPtr[nFing];

	arm=NULL;
	for (int i = 0;i < tips.size();i++) {
		fingers[i]=NULL;
	}
	
	arm = new Arm(bodyItemHuman->body(),base, palm);
	for (int i = 0;i < tips.size();i++) {
		if(!fingers[i]) fingers[i] = new Finger(bodyItemHuman->body(), palm, bodyItemHuman->body()->link(tips[i].toString()) );
		fingers[i]->number = i;
	}
	if( gSettings.find("armStandardPose")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["armStandardPose"].toSequence();
		for(int i=0;i<list.size();i++){
			arm->armStandardPose.push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("fingerOpenPose")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["fingerOpenPose"].toSequence();
		int j=0;
		int k=0;
		for(int i=0;i<list.size();i++){
			if(i>=k+fingers[j]->fing_path->numJoints()){
				k += fingers[j]->fing_path->numJoints();
				j++;
			}
			fingers[j]->fingerOpenPose.push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("fingerOpenPoseOffset")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["fingerOpenPoseOffset"].toSequence();
		int j=0;
		int k=0;
		for(int i=0;i<list.size();i++){
			if(i>=k+fingers[j]->fing_path->numJoints()){
				k += fingers[j]->fing_path->numJoints();
				j++;
			}
			fingers[j]->fingerOpenPoseOffset.push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("fingerOffset")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["fingerOffset"].toSequence();
		for(int j=0; j<nFing; j++)
			fingers[j]->offset = list[0].toDouble();
	}

	vector <InterLink> & interLinkList = PlanBase::instance()->interLinkList;
	if( gSettings.find("interlink")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["interlink"].toSequence();
		for(int i=0;i<list.size();i++){
			const YamlSequence& ilist = *list[i].toSequence();
			Link* master = bodyItemHuman->body()->link(ilist[0].toString());
			double baseratio = ilist[1].toDouble();
			for(int j=1;j<ilist.size()/2;j++){
				InterLink temp;
				temp.master = master;
				temp.slave = bodyItemHuman->body()->link(ilist[2*j].toString());
				temp.ratio = ilist[2*j+1].toDouble()/baseratio;
				interLinkList.push_back(temp);
			}

		}
	}
	if( gSettings.find("approachOffset")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["approachOffset"].toSequence();
		for(int i=0;i<list.size();i++){
			arm->approachOffset[i] = list[i].toDouble();
		}
	}

	if( gSettings.find("selfContactPair")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["selfContactPair"].toSequence();
		if(list[0].type() == YAML_SCALAR){
			for(int i=0;i<list.size()/2;i++){
				contactLinks.insert ( make_pair ( list[i*2].toString(), list[i*2+1].toString() ) );
			}
		}
		if(list[0].type() == YAML_SEQUENCE){
			for(int i=0;i<list.size();i++){
				const YamlSequence& plist = *list[i].toSequence();
				for(int j=0;j<plist.size();j++){
					for(int k=j+1;k<plist.size();k++){
						contactLinks.insert ( make_pair (plist[j].toString(), plist[k].toString() )  );
					}
				}
			}


		}
	}
}

HumanPlanner::HumanPlanner()  : 	os (MessageView::mainInstance()->cout() )
{
	targetHuman = NULL;
}

HumanPlanner::~HumanPlanner() {

}

HumanPlanner* HumanPlanner::instance(HumanPlanner *gc) {
	static HumanPlanner* instance = (gc) ? gc : new HumanPlanner();
	if(gc) instance = gc;
	return instance;
}

void HumanPlanner::SetHuman(BodyItemPtr bodyItem){
	humanList.clear();
	
	//READ YAML setting
	if( bodyItem->body()->info()->find("graspPluginSetting")->type() == YAML_SEQUENCE){ // multi arm
		const YamlSequence& glist = *(*bodyItem->body()->info())["graspPluginSetting"].toSequence();
		for(int i=0;i<glist.size();i++){
			const YamlMapping& gSettings = *glist[i].toMapping();
			if ( gSettings.isValid() && !gSettings.empty()) {
				targetHuman = new Human(bodyItem, gSettings);
				humanList.push_back(targetHuman);
			}
		}
	}
	
	targetHuman = humanList[0];
	
	humanSelfPairs.clear();
	for(unsigned int i=0;i<targetHuman->bodyItemHuman->body()->numLinks();i++){
		for(unsigned int j=i+1;j<targetHuman->bodyItemHuman->body()->numLinks();j++){
			bool pass = false;
			pair<multimap<string, string>::iterator, multimap<string, string>::iterator> ppp;
			ppp = targetHuman->contactLinks.equal_range(targetHuman->bodyItemHuman->body()->link(i)->name() );
			for (multimap<string, string>::iterator it2 = ppp.first; it2 != ppp.second; ++it2){
				if(it2->second == targetHuman->bodyItemHuman->body()->link(j)->name()) pass = true;
			}
			ppp = targetHuman->contactLinks.equal_range(targetHuman->bodyItemHuman->body()->link(j)->name() );
			for (multimap<string, string>::iterator it2 = ppp.first; it2 != ppp.second; ++it2){
				if(it2->second == targetHuman->bodyItemHuman->body()->link(i)->name()) pass = true;
			}
			if(pass) continue;
			ColdetLinkPairPtr temp= make_shared<ColdetLinkPair>(targetHuman->bodyItemHuman->body(),targetHuman->bodyItemHuman->body()->link(i),targetHuman->bodyItemHuman->body(),targetHuman->bodyItemHuman->body()->link(j));
			temp->updatePositions();
			int t1,t2;
			double p1[3],p2[3];
			double distance = temp->computeDistance(t1,p1,t2,p2);
			if(distance>1.0e-04)	humanSelfPairs.push_back(temp);
#ifdef DEBUG_MODE
			else os <<"collide on initial condition at humanSelfPair"  <<distance <<" "<< temp->model(0)->name() <<" " << temp->model(1)->name()  << endl;
#endif
		}
	}
}

void HumanPlanner::setHumanPosition(Vector3 pos, Matrix3 rot){
	if(!targetHuman){
		os<<"Error : Please Set Target Human"<<endl;
		return;
	}
	human()->p() = pos;
	human()->R() = rot;
}

void HumanPlanner::setHumanPose(vector<double> angle){
	if(!targetHuman){
		os<<"Error : Please Set Target Human"<<endl;
		return;
	}
	
	for(int i=0;i<targetHuman->bodyItemHuman->body()->numJoints();i++){
		if(i<angle.size())
			targetHuman->bodyItemHuman->body()->joint(i)->q() = angle[i];
		else
			break;
	}
}

void HumanPlanner::initialCollision(){
	PlanBase* tc = PlanBase::instance();
	tc->initial();
	
	if(targetHuman){
		robotHumanPairs.clear();
		wagonHumanPairs.clear();
		objHumanPairs.clear();
		
		for(unsigned int j=0;j<tc->bodyItemRobot()->body()->numLinks();j++){
			for(unsigned int i=0;i<targetHuman->bodyItemHuman->body()->numLinks();i++){
				if(((tc->bodyItemRobot()->body()->link(j)->name()=="RARM_JOINT7")||(tc->bodyItemRobot()->body()->link(j)->name()=="RARM_JOINTG")||
					(tc->bodyItemRobot()->body()->link(j)->name()=="LARM_JOINT7")||(tc->bodyItemRobot()->body()->link(j)->name()=="LARM_JOINTG"))&&
					((targetHuman->bodyItemHuman->body()->link(i)->name()=="R_ARM_JOINT6")||(targetHuman->bodyItemHuman->body()->link(i)->name()=="R_ARM_JOINT7")||(targetHuman->bodyItemHuman->body()->link(i)->name()=="R_ARM_JOINT8")||
					(targetHuman->bodyItemHuman->body()->link(i)->name()=="L_ARM_JOINT6")||(targetHuman->bodyItemHuman->body()->link(i)->name()=="L_ARM_JOINT7")||(targetHuman->bodyItemHuman->body()->link(i)->name()=="L_ARM_JOINT8")))
					continue;
				robotHumanPairs.push_back(make_shared<ColdetLinkPair>(tc->bodyItemRobot()->body(),tc->bodyItemRobot()->body()->link(j), targetHuman->bodyItemHuman->body(),targetHuman->bodyItemHuman->body()->link(i)));
			}
		}
		
		if(WagonPlanner::instance()->targetWagon){
			for(unsigned int i=0;i<targetHuman->bodyItemHuman->body()->numLinks();i++){
				wagonHumanPairs.push_back(make_shared<ColdetLinkPair>(targetHuman->bodyItemHuman->body(),targetHuman->bodyItemHuman->body()->link(i), WagonPlanner::instance()->targetWagon->wagonItem()->body(),WagonPlanner::instance()->wagon()));
			}
		}
			
		if(tc->targetObject){
			for(unsigned int i=0;i<targetHuman->bodyItemHuman->body()->numLinks();i++){
				if((targetHuman->bodyItemHuman->body()->link(i)->name()=="R_ARM_JOINT6")||(targetHuman->bodyItemHuman->body()->link(i)->name()=="R_ARM_JOINT7")||(targetHuman->bodyItemHuman->body()->link(i)->name()=="R_ARM_JOINT8")||
					(targetHuman->bodyItemHuman->body()->link(i)->name()=="L_ARM_JOINT6")||(targetHuman->bodyItemHuman->body()->link(i)->name()=="L_ARM_JOINT7")||(targetHuman->bodyItemHuman->body()->link(i)->name()=="L_ARM_JOINT8"))
					continue;
				objHumanPairs.push_back(make_shared<ColdetLinkPair>(targetHuman->bodyItemHuman->body(),targetHuman->bodyItemHuman->body()->link(i), tc->targetObject->bodyItemObject->body(),tc->object()));
			}
		}
	}
			
	tc->graspMotionSeq.clear();
}

bool HumanPlanner::isColliding(){
	if(PlanBase::instance()->isColliding())
		return true;
	
	if(targetHuman){
		for(unsigned int i=0;i<humanSelfPairs.size();i++){
			ColdetLinkPairPtr testPair = humanSelfPairs[i];
			testPair->updatePositions();
			bool coll = testPair->checkCollision();
			if(coll){
		#ifdef DEBUG_MODE
				cout <<"human self collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
		#endif
				return true;
			}
		}
		if(PlanBase::instance()->targetArmFinger){
			for(unsigned int i=0;i<robotHumanPairs.size();i++){
				ColdetLinkPairPtr testPair = robotHumanPairs[i];
				testPair->updatePositions();
				bool coll = testPair->checkCollision();
				if(coll){
			#ifdef DEBUG_MODE
					cout <<"robot human collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
			#endif
					return true;
				}
			}
		}
		if(PlanBase::instance()->targetObject){
			for(unsigned int i=0;i<objHumanPairs.size();i++){
				ColdetLinkPairPtr testPair = objHumanPairs[i];
				testPair->updatePositions();
				bool coll = testPair->checkCollision();
				if(coll){
			#ifdef DEBUG_MODE
					cout <<"obj human collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
			#endif
					return true;
				}
			}
		}
		if(WagonPlanner::instance()->targetWagon){
			for(unsigned int i=0;i<wagonHumanPairs.size();i++){
				ColdetLinkPairPtr testPair = wagonHumanPairs[i];
				testPair->updatePositions();
				bool coll = testPair->checkCollision();
				if(coll){
			#ifdef DEBUG_MODE
					cout <<"wagon human collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
			#endif
					return true;
				}
			}
		}
		
	}
	return false;
}

bool HumanPlanner::flush(){
	PlanBase::instance()->calcForwardKinematics();
	PlanBase::instance()->flush();
	
	if(targetHuman){
		targetHuman->bodyItemHuman->body()->calcForwardKinematics();
		targetHuman->bodyItemHuman->notifyKinematicStateChange();
	}
	
	MessageView::mainInstance()->flush();
	return true;
}
