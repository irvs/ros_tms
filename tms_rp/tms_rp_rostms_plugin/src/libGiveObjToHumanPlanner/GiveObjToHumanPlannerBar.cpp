#include "GiveObjToHumanPlannerBar.h"
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/Archive>	/* modified by qtconv.rb 0th rule*/  
#include <boost/bind.hpp>
#include <boost/format.hpp>

#include "./extplugin/graspPlugin/Grasp/PlanBase.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

int GiveObjToHumanPlannerBar::count = 0;


GiveObjToHumanPlannerBar* GiveObjToHumanPlannerBar::instance()
{
	static GiveObjToHumanPlannerBar* instance = new GiveObjToHumanPlannerBar();
	return instance;
}

GiveObjToHumanPlannerBar::GiveObjToHumanPlannerBar()
	: ToolBar("GiveObjToHumanPlannerBar"),
	  mes(*MessageView::mainInstance()),
   	os (MessageView::mainInstance()->cout() )
{
	
	addSeparator();
	
	addLabel(("=GiveObjToHumanPlan="));

	addButton(("SetHuman"), ("Set Human"))->
		sigClicked().connect(bind(&GiveObjToHumanPlannerBar::onSetHumanButtonClicked, this));

	addButton(("MakeManipulabilityMap"), ("Make Manipulability Map"))->
		sigClicked().connect(bind(&GiveObjToHumanPlannerBar::onMakeManipulabilityMapButtonClicked, this));

	addButton(("calcGiveObjPos"), ("calc Give Object to Human Position"))->
		sigClicked().connect(bind(&GiveObjToHumanPlannerBar::oncalcGiveObjPosButtonClicked, this));

	addButton(("calcGetObjPos"), ("calc Get Object Position"))->
		sigClicked().connect(bind(&GiveObjToHumanPlannerBar::oncalcGetObjPosButtonClicked, this));

	addSeparator();
	
	ItemTreeView::mainInstance()->sigSelectionChanged().connect(
		bind(&GiveObjToHumanPlannerBar::onItemSelectionChanged, this, _1));
	count++;
}


GiveObjToHumanPlannerBar::~GiveObjToHumanPlannerBar()
{
	connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
	count--;
}

void GiveObjToHumanPlannerBar::onItemSelectionChanged(const ItemList<BodyItem>& bodyItems)
{
	bool selectedBodyItemsChanged = false;
	if(count < 1) return;
	if(selectedBodyItems_ != bodyItems){
		selectedBodyItems_ = bodyItems;
		selectedBodyItemsChanged = true;
	}

	BodyItemPtr firstItem = bodyItems.toSingle();

	if(firstItem && firstItem != currentBodyItem_){
		currentBodyItem_ = firstItem;
		connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
		connectionOfCurrentBodyItemDetachedFromRoot = currentBodyItem_->sigDetachedFromRoot().connect(
			bind(&GiveObjToHumanPlannerBar::onBodyItemDetachedFromRoot, this));
		sigCurrentBodyItemChanged_(currentBodyItem_.get());
	}

	if(selectedBodyItemsChanged){
		sigBodyItemSelectionChanged_(selectedBodyItems_);
	}

	targetBodyItems.clear();
	if(selectedBodyItems_.empty()){
//		if(currentBodyItem_){
//			targetBodyItems.push_back(currentBodyItem_);
//		}
	} else {
		targetBodyItems = selectedBodyItems_;
	}
}

void GiveObjToHumanPlannerBar::onBodyItemDetachedFromRoot()
{
	currentBodyItem_ = 0;
	connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
	sigCurrentBodyItemChanged_(0);
}

void GiveObjToHumanPlannerBar::onSetHumanButtonClicked()
{
	if(targetBodyItems.size()==1){
		HumanPlanner::instance()->SetHuman(targetBodyItems[0]);
		os << HumanPlanner::instance()->targetHuman->name() << " is target human"<< endl;
		HumanPlanner::instance()->initialCollision();
	}else{
		os <<  "Please selecet one bodyitem" << endl;
		return;
	}
}

void GiveObjToHumanPlannerBar::onMakeManipulabilityMapButtonClicked()
{
	HumanPlanner::instance()->initialCollision();
	GiveObjToHumanPlanner::instance()->makeManipulabilityMap('R', true);
	//~ GiveObjToHumanPlanner::instance()->makeManipulabilityMap('R', false);
	GiveObjToHumanPlanner::instance()->makeManipulabilityMap('H', true);
	//~ GiveObjToHumanPlanner::instance()->makeManipulabilityMap('H', false);
}

void GiveObjToHumanPlannerBar::oncalcGiveObjPosButtonClicked()
{
	//~ os<<HumanPlanner::instance()->targetHuman->bodyItemHuman->body()->link(22)->p<<endl;
	Vector3 human_face_pos = HumanPlanner::instance()->base()->p();
	human_face_pos(2) = HumanPlanner::instance()->targetHuman->bodyItemHuman->body()->link(22)->p()(2);
	GiveObjToHumanPlanner::instance()->unifyManipulabilityMap(human_face_pos, rpyFromRot(HumanPlanner::instance()->base()->R())(2), 160, 'R', true, 'R', false);
	vector<vector<double> > givePosList;
	GiveObjToHumanPlanner::instance()->calcRobotPos_GiveObj(0, givePosList);
	GiveObjToHumanPlanner::instance()->calc_posList_distance(PlanBase::instance()->body()->link(0)->p(), givePosList);
	GiveObjToHumanPlanner::instance()->QSort_posList_distance(givePosList, 0, givePosList.size()-1);
	
	Vector3 PalmPos;
	Matrix3	y_roll, z_roll, palm_R;
	y_roll(0, 0) = 0.0; y_roll(0, 1) = 0.0; y_roll(0, 2) = -1.0;
	y_roll(1, 0) = 0.0; y_roll(1, 1) = 1.0; y_roll(1, 2) = 0.0;
	y_roll(2, 0) = 1.0; y_roll(2, 1) = 0.0; y_roll(2, 2) = 0.0;
	
	//~ for(int i=0;i<givePosList.size();i++){
		//~ PlanBase::instance()->body()->link(0)->p()(0) = givePosList[i][0];
		//~ PlanBase::instance()->body()->link(0)->p()(1) = givePosList[i][1];
		//~ PlanBase::instance()->body()->link(0)->R = rotFromRpy(0,0,givePosList[i][2]);
		//~ PlanBase::instance()->calcForwardKinematics();
		//~ 
		//~ for(int k=0;k<3;k++)
			//~ PalmPos(k) = givePosList[i][k+3];
//~ 
		//~ z_roll(0, 0) = cos(givePosList[i][2]); z_roll(0, 1) = -sin(givePosList[i][2]); z_roll(0, 2) = 0.0;
		//~ z_roll(1, 0) = sin(givePosList[i][2]); z_roll(1, 1) =  cos(givePosList[i][2]); z_roll(1, 2) = 0.0;
		//~ z_roll(2, 0) = 0.0; z_roll(2, 1) = 0.0; z_roll(2, 2) = 1.0;
//~ 
		//~ palm_R = z_roll * y_roll;
		//~ 
		//~ PlanBase::instance()->arm()->IK_arm(PalmPos, palm_R);
		//~ 
		//~ PlanBase::instance()->calcForwardKinematics();
		//~ PlanBase::instance()->flush();sleep(0.1);
	//~ }
	
	//~ for(int i=0;i<givePosList.size();i++){
		//~ PlanBase::instance()->body()->link(0)->p()(0) = givePosList[i][0];
		//~ PlanBase::instance()->body()->link(0)->p()(1) = givePosList[i][1];
		//~ PlanBase::instance()->body()->link(0)->R = rotFromRpy(0,0,givePosList[i][2]);
		//~ PlanBase::instance()->calcForwardKinematics();
		//~ PlanBase::instance()->flush();sleep(0.1);
	//~ }
	//~ GiveObjToHumanPlanner::instance()->expandRobotObjDist(PlanBase::instance()->object()->p, 0.1, givePosList);
	//~ for(int i=0;i<givePosList.size();i++){
		//~ PlanBase::instance()->body()->link(0)->p()(0) = givePosList[i][0];
		//~ PlanBase::instance()->body()->link(0)->p()(1) = givePosList[i][1];
		//~ PlanBase::instance()->body()->link(0)->R = rotFromRpy(0,0,givePosList[i][2]);
		//~ PlanBase::instance()->calcForwardKinematics();
		//~ PlanBase::instance()->flush();sleep(0.1);
	//~ }
	
	//~ vector<vector<vector<double> > > kPosList;
	//~ GiveObjToHumanPlanner::instance()->dividePosList(6, givePosList, kPosList);
	
	vector<vector<double> > freePosList;
	GiveObjToHumanPlanner::instance()->removeCollisionPos(givePosList, freePosList);
	double l = 0.1;
	while(freePosList.empty()){
		GiveObjToHumanPlanner::instance()->expandRobotObjDist(PlanBase::instance()->object()->p(), l, givePosList);
		GiveObjToHumanPlanner::instance()->removeCollisionPos(givePosList, freePosList);
		if(freePosList.empty()){
			if(l<=0.5){
				l+=0.1;
				continue;
			}
			else{
				os<<"Error : can't find free pos"<<endl;
				return;
			}
		}
	}
	
	//~ cout<<givePosList.size()<<"	"<<freePosList.size()<<endl;
	//~ 
	GiveObjToHumanPlanner::instance()->calc_posList_distance(PlanBase::instance()->body()->link(0)->p(), freePosList);
	GiveObjToHumanPlanner::instance()->QSort_posList_distance(freePosList, 0, freePosList.size()-1);
	
	
	//~ for(int i=0;i<freePosList.size();i++){
		//~ PlanBase::instance()->body()->link(0)->p()(0) = freePosList[i][0];
		//~ PlanBase::instance()->body()->link(0)->p()(1) = freePosList[i][1];
		//~ PlanBase::instance()->body()->link(0)->R = rotFromRpy(0,0,freePosList[i][2]);
		//~ PlanBase::instance()->calcForwardKinematics();
		//~ 
		//~ for(int k=0;k<3;k++)
			//~ PalmPos(k) = freePosList[i][k+3];
//~ 
		//~ z_roll(0, 0) = cos(freePosList[i][2]); z_roll(0, 1) = -sin(freePosList[i][2]); z_roll(0, 2) = 0.0;
		//~ z_roll(1, 0) = sin(freePosList[i][2]); z_roll(1, 1) =  cos(freePosList[i][2]); z_roll(1, 2) = 0.0;
		//~ z_roll(2, 0) = 0.0; z_roll(2, 1) = 0.0; z_roll(2, 2) = 1.0;
//~ 
		//~ palm_R = z_roll * y_roll;
		//~ 
		//~ PlanBase::instance()->arm()->IK_arm(PalmPos, palm_R);
		//~ 
		//~ PlanBase::instance()->calcForwardKinematics();
		//~ PlanBase::instance()->flush();sleep(0.1);
	//~ }
	
	PlanBase::instance()->body()->link(0)->p()(0) = freePosList[0][0];
	PlanBase::instance()->body()->link(0)->p()(1) = freePosList[0][1];
	PlanBase::instance()->body()->link(0)->R() = rotFromRpy(0,0,freePosList[0][2]);
	PlanBase::instance()->calcForwardKinematics();
	
	for(int k=0;k<3;k++)
		PalmPos(k) = freePosList[0][k+3];
		
	z_roll(0, 0) = cos(freePosList[0][2]); z_roll(0, 1) = -sin(freePosList[0][2]); z_roll(0, 2) = 0.0;
	z_roll(1, 0) = sin(freePosList[0][2]); z_roll(1, 1) =  cos(freePosList[0][2]); z_roll(1, 2) = 0.0;
	z_roll(2, 0) = 0.0; z_roll(2, 1) = 0.0; z_roll(2, 2) = 1.0;

	palm_R = z_roll * y_roll;
		
	PlanBase::instance()->arm()->IK_arm(PalmPos, palm_R);
	
	HumanPlanner::instance()->flush();
	
}

void GiveObjToHumanPlannerBar::oncalcGetObjPosButtonClicked()
{
	vector<vector<double> > getPosList, freePosList;
	GiveObjToHumanPlanner::instance()->SetRobotManipulabilityMap('R', true);
	GiveObjToHumanPlanner::instance()->calcRobotPos_GetObj(0, getPosList);
	GiveObjToHumanPlanner::instance()->calc_posList_distance(PlanBase::instance()->body()->link(0)->p(), getPosList);
	GiveObjToHumanPlanner::instance()->QSort_posList_distance(getPosList, 0, getPosList.size()-1);
	
	GiveObjToHumanPlanner::instance()->removeCollisionPos(getPosList, freePosList);
	double l = 0.1;
	while(freePosList.empty()){
		GiveObjToHumanPlanner::instance()->expandRobotObjDist(PlanBase::instance()->object()->p(), l, getPosList);
		GiveObjToHumanPlanner::instance()->removeCollisionPos(getPosList, freePosList);
		
		if((!freePosList.empty())||l>=0.3)
			break;
		
		else
			l+=0.1;
	}

	GiveObjToHumanPlanner::instance()->QSort_posList_distance(freePosList, 0, freePosList.size()-1);
	
	PlanBase::instance()->body()->link(0)->p()(0) = freePosList[0][0];
	PlanBase::instance()->body()->link(0)->p()(1) = freePosList[0][1];
	PlanBase::instance()->body()->link(0)->R() = rotFromRpy(0,0,freePosList[0][2]);
	PlanBase::instance()->calcForwardKinematics();
	
	GraspController::instance()->loadAndSelectGraspPattern();
	
	PlanBase::instance()->calcForwardKinematics();
	PlanBase::instance()->flush();
}
