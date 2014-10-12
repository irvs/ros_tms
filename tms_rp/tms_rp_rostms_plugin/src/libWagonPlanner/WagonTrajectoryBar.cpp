#include "WagonTrajectoryBar.h"
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <boost/bind.hpp>
#include <boost/format.hpp>

#include "./extplugin/graspPlugin/Grasp/GraspController.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;



SetMapLimitDialog::SetMapLimitDialog() : QDialog(MainWindow::instance()) {
	
	setWindowTitle("Set Map Limit");
	
	QVBoxLayout* vbox = new QVBoxLayout();
	setLayout(vbox);
	
	QHBoxLayout* hbox;
	
	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("llimit x"));
	llimit_x.setAlignment(Qt::AlignCenter);
	llimit_x.setDecimals(3);
	llimit_x.setRange(-5.000, 5.000);
	llimit_x.setSingleStep(0.001);
	llimit_x.setValue(0.000);
	//~ llimit_x.setValue(PlanBase::instance()->llimitMap[0]);
	hbox->addWidget(&llimit_x);
	hbox->addWidget(new QLabel("(m)"));
	hbox->addStretch();
	vbox->addLayout(hbox);
	
	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("llimit y"));
	llimit_y.setAlignment(Qt::AlignCenter);
	llimit_y.setDecimals(3);
	llimit_y.setRange(-5.000, 5.000);
	llimit_y.setSingleStep(0.001);
	llimit_y.setValue(0.000);
	//~ llimit_y.setValue(PlanBase::instance()->llimitMap[1]);
	hbox->addWidget(&llimit_y);
	hbox->addWidget(new QLabel("(m)"));
	hbox->addStretch();
	vbox->addLayout(hbox);
	
	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("llimit z"));
	llimit_z.setAlignment(Qt::AlignCenter);
	llimit_z.setDecimals(3);
	llimit_z.setRange(-5.000, 5.000);
	llimit_z.setSingleStep(0.001);
	llimit_z.setValue(-0.500);
	//~ llimit_z.setValue(PlanBase::instance()->llimitMap[2]);
	hbox->addWidget(&llimit_z);
	hbox->addWidget(new QLabel("(m)"));
	hbox->addStretch();
	vbox->addLayout(hbox);
	
	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("ulimit x"));
	ulimit_x.setAlignment(Qt::AlignCenter);
	ulimit_x.setDecimals(3);
	ulimit_x.setRange(-5.000, 5.000);
	ulimit_x.setSingleStep(0.001);
	ulimit_x.setValue(4.500);
	//~ ulimit_x.setValue(PlanBase::instance()->ulimitMap[0]);
	hbox->addWidget(&ulimit_x);
	hbox->addWidget(new QLabel("(m)"));
	hbox->addStretch();
	vbox->addLayout(hbox);
	
	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("ulimit y"));
	ulimit_y.setAlignment(Qt::AlignCenter);
	ulimit_y.setDecimals(3);
	ulimit_y.setRange(-5.000, 5.000);
	ulimit_y.setSingleStep(0.001);
	ulimit_y.setValue(4.000);
	//~ ulimit_y.setValue(PlanBase::instance()->ulimitMap[1]);
	hbox->addWidget(&ulimit_y);
	hbox->addWidget(new QLabel("(m)"));
	hbox->addStretch();
	vbox->addLayout(hbox);
	
	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("ulimit z"));
	ulimit_z.setAlignment(Qt::AlignCenter);
	ulimit_z.setDecimals(3);
	ulimit_z.setRange(-5.000, 5.000);
	ulimit_z.setSingleStep(0.001);
	ulimit_z.setValue(0.500);
	//~ ulimit_z.setValue(PlanBase::instance()->ulimitMap[2]);
	hbox->addWidget(&ulimit_z);
	hbox->addWidget(new QLabel("(m)"));
	hbox->addStretch();
	vbox->addLayout(hbox);
	

	cnoid::PushButton* okButton = new cnoid::PushButton("&OK");
	okButton->setDefault(true);
	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
	okButton->sigClicked().connect(boost::bind(&SetMapLimitDialog::okClicked, this));
	
	vbox->addWidget(okButton);
}

void SetMapLimitDialog::okClicked(){
	PlanBase::instance()->llimitMap[0] = llimit_x.value();
	PlanBase::instance()->llimitMap[1] = llimit_y.value();
	PlanBase::instance()->llimitMap[2] = llimit_z.value();
	PlanBase::instance()->ulimitMap[0] = ulimit_x.value();
	PlanBase::instance()->ulimitMap[1] = ulimit_y.value();
	PlanBase::instance()->ulimitMap[2] = ulimit_z.value();
	MessageView::mainInstance()->cout() << "Set Map Limit"<<endl;
	MessageView::mainInstance()->cout() <<"llimit:"<<PlanBase::instance()->llimitMap[0]<<" "<<PlanBase::instance()->llimitMap[1]<<" "<<PlanBase::instance()->llimitMap[2]<< endl;
	MessageView::mainInstance()->cout() <<"ulimit:"<<PlanBase::instance()->ulimitMap[0]<<" "<<PlanBase::instance()->ulimitMap[1]<<" "<<PlanBase::instance()->ulimitMap[2]<< endl;
}

WagonTrajectoryBar* WagonTrajectoryBar::instance()
{
	static WagonTrajectoryBar* instance = new WagonTrajectoryBar();
	return instance;
}

WagonTrajectoryBar::WagonTrajectoryBar()
	: ToolBar("WagonTrajectoryBar"),
	  mes(*MessageView::mainInstance()),
   	os (MessageView::mainInstance()->cout() )
{
	
	addSeparator();
	
	addLabel(("=WagonPathPlan="));

	addButton(("SetMapLimit"), ("Set Map Limit"))->
		sigClicked().connect(bind(&WagonTrajectoryBar::onSetMapLimitButtonClicked, this));

	addButton(("StartPRMPlanning"), ("Path planning start"))->
		sigClicked().connect(bind(&WagonTrajectoryBar::onWagonTrajectoryPlanButtonClicked, this));
		
	addSeparator();

	addButton(("ResetStates"), ("motion states reset"))->
		sigClicked().connect(bind(&WagonTrajectoryBar::onResetButtonClicked, this));	 
	
	addButton(("setStartState"), ("Set start Motion"))->
		sigClicked().connect(bind(&WagonTrajectoryBar::onSetStartMotionStateButtonClicked, this));	 

	addButton(("AddMiddleMotionState"), ("Add Motion State"))->
		sigClicked().connect(bind(&WagonTrajectoryBar::onAddMiddleMotionStateButtonClicked, this)); 
	
	addButton(("setEndState"), ("Set End Motion"))->
		sigClicked().connect(bind(&WagonTrajectoryBar::onSetEndMotionStateButtonClicked, this));	  
	
	addButton(("CheckMotionState"), ("Check Motion State"))->
		sigClicked().connect(bind(&WagonTrajectoryBar::onCheckMotionStateButtonClicked, this));  

	addSeparator();
	
	addButton(("WagonGraspPlanning"), ("Grasp wagon planning start"))->
		sigClicked().connect(bind(&WagonTrajectoryBar::onWagonGraspButtonClicked, this));	  

	addButton(("WagonReleasePlanning"), ("Release wagon planning start"))->
		sigClicked().connect(bind(&WagonTrajectoryBar::onWagonReleaseButtonClicked, this));	  

	//~ addButton(("WagonPushVoronoiPlanning"), ("Push wagon path on voronoi planning start"))->
		//~ sigClicked().connect(bind(&WagonTrajectoryBar::onWagonPushPlanButtonClicked, this));	  

}


WagonTrajectoryBar::~WagonTrajectoryBar()
{
}


void WagonTrajectoryBar::setPlanDOF(std::vector<WagonMotionState>& PlanMotionState){
	PlanBase* pb = PlanBase::instance();
	int top = pb->body()->numJoints();
	if(PlanMotionState.size()<1){
		os<<"Please Add Plan Motion"<<endl;
		return;
	}
	PlanMotionState[0].pathPlanDOF.clear();
	for(int j=0;j<pb->body()->numJoints();j++){
		PlanMotionState[0].pathPlanDOF.push_back(pb->body()->joint(j)->jointId());
	}
	PlanMotionState[0].pathPlanDOF.push_back(top);
	PlanMotionState[0].pathPlanDOF.push_back(top+1);
	PlanMotionState[0].pathPlanDOF.push_back(top+5);
	
	if(PlanMotionState.size()<2)
		return;
		
	for(unsigned int i=0;i<PlanMotionState.size()-1;i++){
		PlanMotionState[i].pathPlanDOF.clear();
		//for SmartPal5
		int start_id, end_id;
		double d_angle;
		//WAIST
		start_id = 0, end_id = 2;
		d_angle = 0.0;
		for(int j=start_id;j<end_id;j++)
			d_angle += fabs(PlanMotionState[i+1].jointSeq(j) - PlanMotionState[i].jointSeq(j));
		if( d_angle > 1.e-10)
			for(int j=start_id;j<end_id;j++)
				PlanMotionState[i].pathPlanDOF.push_back(pb->body()->joint(j)->jointId());
		//Arm_R		
		start_id = end_id, end_id = 9;
		d_angle = 0.0;
		for(int j=start_id;j<end_id;j++)
			d_angle += fabs(PlanMotionState[i+1].jointSeq(j) - PlanMotionState[i].jointSeq(j));
		if( d_angle > 1.e-10)
			for(int j=start_id;j<end_id;j++)
				PlanMotionState[i].pathPlanDOF.push_back(pb->body()->joint(j)->jointId());
				
		//Gripper_R
		d_angle = 0.0;
		d_angle = fabs(PlanMotionState[i+1].jointSeq(end_id) - PlanMotionState[i].jointSeq(end_id));
		if( d_angle > 1.e-10)
			PlanMotionState[i].pathPlanDOF.push_back(pb->body()->joint(end_id)->jointId());
		end_id = 10;
				
		//Arm_L	
		start_id = end_id, end_id = 17;
		d_angle = 0.0;
		for(int j=start_id;j<end_id;j++)
			d_angle += fabs(PlanMotionState[i+1].jointSeq(j) - PlanMotionState[i].jointSeq(j));
		if( d_angle > 1.e-10)
			for(int j=start_id;j<end_id;j++)
				PlanMotionState[i].pathPlanDOF.push_back(pb->body()->joint(j)->jointId());
				
		//Gripper_L
		d_angle = 0.0;
		d_angle = fabs(PlanMotionState[i+1].jointSeq(end_id) - PlanMotionState[i].jointSeq(end_id));
		if( d_angle > 1.e-10)
			PlanMotionState[i].pathPlanDOF.push_back(pb->body()->joint(end_id)->jointId());
		end_id = 18;
		
		//Base
		if( (PlanMotionState[i].pos - PlanMotionState[i+1].pos).norm() > 1.e-10){
			PlanMotionState[i].pathPlanDOF.push_back(top);
			PlanMotionState[i].pathPlanDOF.push_back(top+1);
			PlanMotionState[i].pathPlanDOF.push_back(top+5);
		}
		
		//~ cout<<PlanMotionState[i].pathPlanDOF.size()<<endl;
	}
	
	PlanMotionState[PlanMotionState.size()-1].pathPlanDOF.clear();
	for(int j=0;j<pb->body()->numJoints();j++){
		PlanMotionState[PlanMotionState.size()-1].pathPlanDOF.push_back(pb->body()->joint(j)->jointId());
	}
	PlanMotionState[PlanMotionState.size()-1].pathPlanDOF.push_back(top);
	PlanMotionState[PlanMotionState.size()-1].pathPlanDOF.push_back(top+1);
	PlanMotionState[PlanMotionState.size()-1].pathPlanDOF.push_back(top+5);
	
}

void WagonTrajectoryBar::onSetMapLimitButtonClicked()
{
	SetMapLimitDialog* SMLDialog = new SetMapLimitDialog();
	SMLDialog->show();
}

void WagonTrajectoryBar::onWagonTrajectoryPlanButtonClicked()
{
	PlanBase* tc = PlanBase::instance();
	WagonPlanner* wp = WagonPlanner::instance();
	if(!tc->targetArmFinger){
		os << "error: Set Robot" << endl;
		return;
	}
	
	wp->wagonMotionSeq.clear();
	if(wp->startMotionState.id > 0){
		wp->wagonMotionSeq.push_back(wp->startMotionState);
		wp->startMotionState.id = -1;
	}
	for(unsigned int i=0;i<tempMotionSeq.size();i++){
		wp->wagonMotionSeq.push_back(tempMotionSeq[i]);
	}
	if(wp->endMotionState.id > 0){
		wp->wagonMotionSeq.push_back(wp->endMotionState);
		wp->endMotionState.id = -1;
	}
	
	setPlanDOF(wp->wagonMotionSeq);
	
	WagontrajectoryPlanner_ = new WagonTrajectoryPlanner();

	bool ret = WagontrajectoryPlanner_->doWagonTrajectoryPlanning();

	if(ret)
			os <<  "Trajectory Planning is finished" << endl;	
	else
			os <<  "Trajectory Planning is failed" << endl;	
}

void WagonTrajectoryBar::onResetButtonClicked()
{
	WagonPlanner* wp = WagonPlanner::instance();
	wp->changeType(wp->targetWagon, InterObject::COLLISION);
	wp->wagonMotionSeq.clear();
	tempMotionSeq.clear();
	wp->startMotionState.id = -1;
	wp->stopWagonMotionState.id = -1;
	wp->endMotionState.id = -1;
	os << "Motion states clear" << endl;
}

void WagonTrajectoryBar::onSetStartMotionStateButtonClicked(){
	WagonPlanner* wp = WagonPlanner::instance();
	if(!PlanBase::instance()->targetArmFinger){
		os << "error: Set Robot" << endl;
		return;
	}
	wp->wagonMotionSeq.clear();
	wp->startMotionState = wp->getMotionState();
	wp->startMotionState.id = 1;
}

void WagonTrajectoryBar::onAddMiddleMotionStateButtonClicked(){
	if(!PlanBase::instance()->targetArmFinger){
		os << "error: Set Robot" << endl;
		return;
	}
	
	//~ PlanBase::instance()->graspMotionSeq.push_back(PlanBase::instance()->getMotionState());
	tempMotionSeq.push_back(WagonPlanner::instance()->getMotionState());
}

void WagonTrajectoryBar::onSetEndMotionStateButtonClicked(){
	WagonPlanner* wp = WagonPlanner::instance();
	if(!PlanBase::instance()->targetArmFinger){
		os << "error: Set Robot" << endl;
		return;
	}
	wp->wagonMotionSeq.clear();
	wp->endMotionState = wp->getMotionState();
	wp->endMotionState.id = 2;
}

void WagonTrajectoryBar::onCheckMotionStateButtonClicked(){
	WagonPlanner* wp = WagonPlanner::instance();
	wp->wagonMotionSeq.clear();
	if(wp->startMotionState.id > 0){
		wp->wagonMotionSeq.push_back(wp->startMotionState);
	}
	for(unsigned int i=0;i<tempMotionSeq.size();i++){
		wp->wagonMotionSeq.push_back(tempMotionSeq[i]);
	}
	if(wp->endMotionState.id > 0){
		wp->wagonMotionSeq.push_back(wp->endMotionState);
	}
	if(wp->wagonMotionSeq.size()<1){
		os << "error: Set Motion State" << endl;
		return;
	}
	for(unsigned int i=0;i<wp->wagonMotionSeq.size();i++){
		wp->setMotionState(wp->wagonMotionSeq[i]);
		PlanBase::instance()->calcForwardKinematics();
		PlanBase::instance()->flush();
		os<<"State No."<<i<<"	isColliding:"<<PlanBase::instance()->isColliding()<<endl;
		sleep(1);
	}
	wp->setMotionState(wp->wagonMotionSeq[0]);
	PlanBase::instance()->calcForwardKinematics();
	PlanBase::instance()->flush();
	os<<"Finish checking."<<endl;
}

void WagonTrajectoryBar::onWagonGraspButtonClicked(){
	PlanBase* tc = PlanBase::instance();
	WagonPlanner* wp = WagonPlanner::instance();
	
	std::vector<WagonMotionState> WagonGraspMotionSeq;
	
	WagonMotionState preMotionState = wp->getMotionState();	
	if(wp->startMotionState.id > 0)
		preMotionState = wp->startMotionState;
	if(tempMotionSeq.size()>0)
		preMotionState = tempMotionSeq[tempMotionSeq.size()-1];
	WagonPushPlanner::instance()->WagonGraspPlan(preMotionState , WagonGraspMotionSeq);
	for(unsigned int i=0;i<WagonGraspMotionSeq.size();i++){
		tempMotionSeq.push_back(WagonGraspMotionSeq[i]);
	}
	
	//~ WagonPushPlanner::instance()->WagonReleasePlan(tempMotionSeq[tempMotionSeq.size()-1] , WagonGraspMotionSeq);
	//~ for(unsigned int i=0;i<WagonGraspMotionSeq.size();i++){
		//~ tempMotionSeq.push_back(WagonGraspMotionSeq[i]);
	//~ }
}

void WagonTrajectoryBar::onWagonReleaseButtonClicked(){
	PlanBase* tc = PlanBase::instance();
	WagonPlanner* wp = WagonPlanner::instance();
	
	std::vector<WagonMotionState> WagonGraspMotionSeq;
	
	tc->calcForwardKinematics();
	tc->flush();
	WagonMotionState preMotionState = wp->getMotionState();	
	//~ if(wp->startMotionState.id > 0)
		//~ preMotionState = wp->startMotionState;
	//~ if(tempMotionSeq.size()>0)
		//~ preMotionState = tempMotionSeq[tempMotionSeq.size()-1];
	WagonPushPlanner::instance()->WagonReleasePlan(preMotionState , WagonGraspMotionSeq);
	for(unsigned int i=0;i<WagonGraspMotionSeq.size();i++){
		tempMotionSeq.push_back(WagonGraspMotionSeq[i]);
	}
	
}

/*
void WagonTrajectoryBar::onWagonPushPlanButtonClicked(){
	PlanBase* tc = PlanBase::instance();
	vector<vector<double> > plan_path;
	WagonPushPlanner::instance()->calcWagonPushPath(VoronoiPathPlanner::instance()->collisionMap, VoronoiPathPlanner::instance()->start_pos, VoronoiPathPlanner::instance()->goal_pos, plan_path);
	//~ vector<double> Rpos, Conpos, Pose;
	//~ Rpos.resize(3);
	//~ Conpos.resize(3);
	//~ Pose.clear();
	//~ Rpos[0] = tc->body()->link(0)->p()(0);
	//~ Rpos[1] = tc->body()->link(0)->p()(1);
	//~ Rpos[2] = rpyFromRot(tc->body()->link(0)->R())(2);
	//~ Conpos[0] = WagonPlanner::instance()->targetWagon->controlDist;
	//~ Conpos[1] = 0.0;
	//~ Conpos[2] = WagonPlanner::instance()->targetWagon->graspHeight;
	//~ 
	//~ double rel_th = deg2rad(-30.0);
	//~ while(1){
		//~ WagonPushPlanner::instance()->calcRelativeWagonGraspPose(Rpos, 3, Conpos, rel_th, Pose);
		//~ rel_th += deg2rad(1.0);
		//~ Rpos[0]+=0.01;
		//~ if(rel_th>deg2rad(30.0))
			//~ break;
	//~ }
}
*/
