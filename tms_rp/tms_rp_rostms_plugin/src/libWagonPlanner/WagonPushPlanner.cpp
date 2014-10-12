#include "WagonPushPlanner.h"
#include "./extplugin/graspPlugin/Grasp/PlanBase.h"
#include "./extplugin/graspPlugin/Grasp/VectorMath.h"
#include <cnoid/MessageView>

using namespace std;
using namespace cnoid;
using namespace grasp;

const double open_gripper_angle = -40.0, close_gripper_angle = 0.0;

WagonPushPlanner::WagonPushPlanner()  : 	os (MessageView::mainInstance()->cout() ){
}

WagonPushPlanner::~WagonPushPlanner() {
}

WagonPushPlanner* WagonPushPlanner::instance(WagonPushPlanner *gc) {
	static WagonPushPlanner* instance = (gc) ? gc : new WagonPushPlanner();
	if(gc) instance = gc;
	return instance;
}


void WagonPushPlanner::setStandardRobotPose(){
	PlanBase* tc = PlanBase::instance();
	tc->bodyItemRobot()->setPresetPose(BodyItem::STANDARD_POSE);
}

void WagonPushPlanner::setRobotPose(vector<double> pose){
	PlanBase* tc = PlanBase::instance();
	for(int i=0;i<tc->body()->numJoints();i++){
		if(i<pose.size())
			tc->body()->joint(i)->q()=pose[i];
		else
			break;
	}
	tc->calcForwardKinematics();
	tc->flush();
}

void WagonPushPlanner::calcWagonPolePos(vector<vector<double> >& out_pos){
	out_pos.clear();
	
	robot = PlanBase::instance()->targetArmFinger;
	rl = PlanBase::instance()->base();
	
	wagon = WagonPlanner::instance()->targetWagon;
	wl = WagonPlanner::instance()->wagon();
	
	wth = rpyFromRot(wl->R())(2);
	
	vector<double>	tempPos;
	vector<vector<double> >	polePos;
	tempPos.resize(2);
	polePos.clear();
	
	tempPos[0] = wagon->size_LongSide_Length/2.0;
	tempPos[1] = wagon->size_ShortSide_Length/2.0;
	polePos.push_back(tempPos);
	
	tempPos[0] *= -1;
	polePos.push_back(tempPos);
	
	tempPos[1] *= -1;
	polePos.push_back(tempPos);
	
	tempPos[0] *= -1;
	polePos.push_back(tempPos);
	
	for(unsigned int i=0;i<polePos.size();i++){
		tempPos[0] = polePos[i][0]*cos(wth) - polePos[i][1]*sin(wth);
		tempPos[1] = polePos[i][0]*sin(wth) + polePos[i][1]*cos(wth);
		tempPos[0] += wl->p()(0);
		tempPos[1] += wl->p()(1);
		polePos[i] = tempPos;
		//~ cout<<polePos[i][0]<<"	"<<polePos[i][1]<<endl;
	}
	
	out_pos = polePos;
	polePos.clear();
}

void WagonPushPlanner::calcWagonGraspBasePos(double distance, vector<vector<double> >& out_pos){
	out_pos.clear();
	
	robot = PlanBase::instance()->targetArmFinger;
	rl = PlanBase::instance()->base();
	
	wagon = WagonPlanner::instance()->targetWagon;
	wl = WagonPlanner::instance()->wagon();
	
	wth = rpyFromRot(wl->R())(2);
	
	vector<double>	tempPos;
	vector<vector<double> >	basePos;
	tempPos.resize(3);
	basePos.clear();
	
	for(int i=0;i<4;i++){
		if(i%2==0){
			tempPos[0] = wl->p()(0) + ( wagon->size_LongSide_Length/2.0 + distance )*cos( wth + (i/2.0)*M_PI );
			tempPos[1] = wl->p()(1) + ( wagon->size_LongSide_Length/2.0 + distance )*sin( wth + (i/2.0)*M_PI );
		}
		else{
			tempPos[0] = wl->p()(0) + ( wagon->size_ShortSide_Length/2.0 + distance )*cos( wth + (i/2.0)*M_PI );
			tempPos[1] = wl->p()(1) + ( wagon->size_ShortSide_Length/2.0 + distance )*sin( wth + (i/2.0)*M_PI );
		}
		tempPos[2] = wth + (2+i)*M_PI/2.0;
		if(tempPos[2] > M_PI)
			tempPos[2] -= 2*M_PI;
		if(tempPos[2] < -M_PI)
			tempPos[2] += 2*M_PI;
		
		basePos.push_back(tempPos);
	}
	
	//~ for(int i=0;i<basePos.size();i++)
		//~ cout<<basePos[i][0]<<"	"<<basePos[i][1]<<"	"<<rad2deg(basePos[i][2])<<endl;
		
	out_pos = basePos;
	basePos.clear();
}

bool WagonPushPlanner::calcWagonGraspPose(vector<double> robot_pos, double& out_palm_th, vector<double>& out_rel_control_pos, double& out_rel_th, vector<double>& out_pose){
	bool RisWagon = true;	//true : palm_R adjust Wagon_th  false : adjust Robot_th
	
	out_pose.clear();
	out_rel_control_pos.resize(3);
	out_rel_control_pos[2] = WagonPlanner::instance()->targetWagon->rel_control_point[2];
	
	WagonPlanner* wp = WagonPlanner::instance();
	int wagon_type = wp->targetWagon->wagon_type;
	wp->changeWagonType(wp->targetWagon, Wagon::COLLISION);
	
	PlanBase* tc = PlanBase::instance();
	
	tc->base()->p()[0] = robot_pos[0];
	tc->base()->p()[1] = robot_pos[1];
	tc->base()->R() = rotFromRpy(0,0,robot_pos[2]);
	
	tc->calcForwardKinematics();
	tc->flush();
	
	wth = rpyFromRot(wp->wagon()->R())(2);
	
	Vector3 palmPos_R, palmPos_L;
	vector<vector<double> >	pole_pos;
	calcWagonPolePos(pole_pos);
	
	double rw_th = atan2( robot_pos[1] - wp->wagon()->p()(1) , robot_pos[0] - wp->wagon()->p()(0) );
	
	vector<double> pw_th;
	pw_th.resize(2);
	
	pw_th[0] = atan2(wp->targetWagon->size_ShortSide_Length/2.0, wp->targetWagon->size_LongSide_Length/2.0 );
	pw_th[1] = atan2(wp->targetWagon->size_LongSide_Length/2.0, wp->targetWagon->size_ShortSide_Length/2.0 );
	
	double dth = rw_th - wth;
	if(dth > M_PI)
		dth -= 2*M_PI;
	if(dth < -M_PI)
		dth += 2*M_PI;
	
	if((dth>=-pw_th[0])&&(dth<=pw_th[0])){
		wp->targetWagon->state = 1;
		
		palmPos_R[0] = pole_pos[0][0];
		palmPos_R[1] = pole_pos[0][1];
		palmPos_R[2] = wp->targetWagon->graspHeight;
		palmPos_L[0] = pole_pos[3][0];
		palmPos_L[1] = pole_pos[3][1];
		palmPos_L[2] = wp->targetWagon->graspHeight;
	}
	else if((dth>-pw_th[1]+(M_PI/2))&&(dth<pw_th[1]+(M_PI/2))){
		wp->targetWagon->state = 2;
		
		palmPos_R[0] = pole_pos[1][0];
		palmPos_R[1] = pole_pos[1][1];
		palmPos_R[2] = wp->targetWagon->graspHeight;
		palmPos_L[0] = pole_pos[0][0];
		palmPos_L[1] = pole_pos[0][1];
		palmPos_L[2] = wp->targetWagon->graspHeight;
	}
	else if(((dth>=-M_PI)&&(dth<=pw_th[0]-M_PI))||((dth>=-pw_th[0]+M_PI)&&(dth<=M_PI))){
		wp->targetWagon->state = 3;
		
		palmPos_R[0] = pole_pos[2][0];
		palmPos_R[1] = pole_pos[2][1];
		palmPos_R[2] = wp->targetWagon->graspHeight;
		palmPos_L[0] = pole_pos[1][0];
		palmPos_L[1] = pole_pos[1][1];
		palmPos_L[2] = wp->targetWagon->graspHeight;
	}
	else if((dth>-pw_th[1]-(M_PI/2))&&(dth<pw_th[1]-(M_PI/2))){
		wp->targetWagon->state = 4;
		
		palmPos_R[0] = pole_pos[3][0];
		palmPos_R[1] = pole_pos[3][1];
		palmPos_R[2] = wp->targetWagon->graspHeight;
		palmPos_L[0] = pole_pos[2][0];
		palmPos_L[1] = pole_pos[2][1];
		palmPos_L[2] = wp->targetWagon->graspHeight;
	}
	
	double control_pos[2] = {0.0, 0.0};
	double control_dist = 0.0;
	for(int i=0;i<2;i++)
		control_pos[i] = ( palmPos_R[i] + palmPos_L[i] ) / 2.0;
	double control_th = atan2( control_pos[1]-robot_pos[1], control_pos[0]-robot_pos[0] ) - robot_pos[2];
	if(control_th > M_PI)
		control_th -= 2*M_PI;
	if(control_th < -M_PI)
		control_th += 2*M_PI;
	control_dist = sqrt( (robot_pos[0]-control_pos[0])*(robot_pos[0]-control_pos[0]) + (robot_pos[1]-control_pos[1])*(robot_pos[1]-control_pos[1]) );
	out_rel_control_pos[0] = control_dist * cos(control_th);
	out_rel_control_pos[1] = control_dist * sin(control_th);
	
	double temp_th = atan2( palmPos_R[1]-palmPos_L[1], palmPos_R[0]-palmPos_L[0] ) + (M_PI/2.0);
	if(temp_th > M_PI)
		temp_th -= 2*M_PI;
	if(temp_th < -M_PI)
		temp_th += 2*M_PI;
		
	out_rel_th = temp_th - robot_pos[2];
	if(out_rel_th > M_PI)
		out_rel_th -= 2*M_PI;
	if(out_rel_th < -M_PI)
		out_rel_th += 2*M_PI;
		
	out_palm_th = robot_pos[2]+(out_rel_th*RisWagon);
		
	palmPos_R[0] += wp->targetWagon->graspPos_offset[0]*cos(out_palm_th) - (-wp->targetWagon->graspPos_offset[1])*sin(out_palm_th);
	palmPos_R[1] += wp->targetWagon->graspPos_offset[0]*sin(out_palm_th) + (-wp->targetWagon->graspPos_offset[1])*cos(out_palm_th);
	palmPos_R[2] += wp->targetWagon->graspPos_offset[2];
	palmPos_L[0] += wp->targetWagon->graspPos_offset[0]*cos(out_palm_th) - wp->targetWagon->graspPos_offset[1]*sin(out_palm_th);
	palmPos_L[1] += wp->targetWagon->graspPos_offset[0]*sin(out_palm_th) + wp->targetWagon->graspPos_offset[1]*cos(out_palm_th);
	palmPos_L[2] += wp->targetWagon->graspPos_offset[2];
	
	Matrix3	yR, zR, palmR;
	yR(0, 0) = 0.0; yR(0, 1) = 0.0; yR(0, 2) = -1.0;
	yR(1, 0) = 0.0; yR(1, 1) = 1.0; yR(1, 2) = 0.0;
	yR(2, 0) = 1.0; yR(2, 1) = 0.0; yR(2, 2) = 0.0;
	
	zR(0, 0) = cos(out_palm_th); zR(0, 1) = -sin(out_palm_th); zR(0, 2) = 0.0;
	zR(1, 0) = sin(out_palm_th); zR(1, 1) =  cos(out_palm_th); zR(1, 2) = 0.0;
	zR(2, 0) = 0.0; zR(2, 1) = 0.0; zR(2, 2) = 1.0;
	
	palmR = zR * yR;
	
	if(!tc->arm(2)->IK_arm(palmPos_R, palmR))
		return false;
	if(!tc->arm(3)->IK_arm(palmPos_L, palmR))
		return false;
	tc->fingers(2, 0)->joint(0)->q() = deg2rad(open_gripper_angle);
	tc->fingers(3, 0)->joint(0)->q() = deg2rad(open_gripper_angle);
	
	tc->calcForwardKinematics();
	tc->flush();
	//~ sleep(1);
	
	//~ WagonPlanner::instance()->changeType(wagon, InterObject::GRASPED_OBJECT);
	//~ WagonPlanner::instance()->flush();
	//~ os<<WagonPlanner::instance()->isColliding()<<endl;
	
	if(WagonPlanner::instance()->isColliding()){
		return false;
	}
		
	for(int i=0;i<tc->bodyItemRobot()->body()->numJoints();i++){
		out_pose.push_back(tc->body()->joint(i)->q());
	}
	
	wp->changeWagonType(wp->targetWagon, Wagon::WagonType(wagon_type));
	return true;
}

bool WagonPushPlanner::calcStandardWagonGraspPose(vector<double> robot_pos, int grasp_type, vector<double>& out_pose){
	out_pose.clear();
	
	robot = PlanBase::instance()->targetArmFinger;
	rl = PlanBase::instance()->base();
	
	wagon = WagonPlanner::instance()->targetWagon;
	wl = WagonPlanner::instance()->wagon();
	
	WagonPlanner* wp = WagonPlanner::instance();
	int wagon_type = wp->targetWagon->wagon_type;
	wp->changeWagonType(wp->targetWagon, Wagon::COLLISION);
	
	int i = grasp_type - 1;
	PlanBase* tc = PlanBase::instance();
	
	tc->base()->p()[0] = robot_pos[0];
	tc->base()->p()[1] = robot_pos[1];
	tc->base()->R() = rotFromRpy(0,0,robot_pos[2]);
	
	tc->calcForwardKinematics();
	tc->flush();
	
	vector<double> tempPos;
	tempPos.resize(3);
	switch(i){
		case 0:
			tempPos[0] = robot_pos[0] + ( wagon->size_LongSide_Length/2.0 + wagon->controlDist )*cos( robot_pos[2] );
			tempPos[1] = robot_pos[1] + ( wagon->size_LongSide_Length/2.0 + wagon->controlDist )*sin( robot_pos[2] );
			tempPos[2] = robot_pos[2] + M_PI;
			break;
	
		case 1:
			tempPos[0] = robot_pos[0] + ( wagon->size_ShortSide_Length/2.0 + wagon->controlDist )*cos( robot_pos[2] );
			tempPos[1] = robot_pos[1] + ( wagon->size_ShortSide_Length/2.0 + wagon->controlDist )*sin( robot_pos[2] );
			tempPos[2] = robot_pos[2] + M_PI/2.0;
			break;
	
		case 2:
			tempPos[0] = robot_pos[0] + ( wagon->size_LongSide_Length/2.0 + wagon->controlDist )*cos( robot_pos[2] );
			tempPos[1] = robot_pos[1] + ( wagon->size_LongSide_Length/2.0 + wagon->controlDist )*sin( robot_pos[2] );
			tempPos[2] = robot_pos[2];
			break;
	
		case 3:
			tempPos[0] = robot_pos[0] + ( wagon->size_ShortSide_Length/2.0 + wagon->controlDist )*cos( robot_pos[2] );
			tempPos[1] = robot_pos[1] + ( wagon->size_ShortSide_Length/2.0 + wagon->controlDist )*sin( robot_pos[2] );
			tempPos[2] = robot_pos[2] - M_PI/2.0;
			break;
	}
	
	wl->p()(0) = tempPos[0];
	wl->p()(1) = tempPos[1];
	wl->R() = rotFromRpy(0,0,tempPos[2]);
	
	wagon->slaveItem->body()->calcForwardKinematics();
	wagon->slaveItem->notifyKinematicStateChange();
	tc->calcForwardKinematics();
	
	vector<double> out_control_point;
	double out_palm_th, out_rel_th;
	calcWagonGraspPose(robot_pos, out_palm_th, out_control_point, out_rel_th, out_pose);
	
	tc->flush();
	wp->changeWagonType(wp->targetWagon, Wagon::WagonType(wagon_type));
	return true;
};

bool WagonPushPlanner::calcRelativeWagonGraspPose(vector<double> robot_pos, int grasp_type, vector<double> rel_control_pos, double rel_th, vector<double>& out_pose){
	out_pose.clear();
	
	WagonPlanner* wp = WagonPlanner::instance();
	wagon = wp->targetWagon;
	wl = wp->wagon();
	int wagon_type = wp->targetWagon->wagon_type;
	wp->changeWagonType(wp->targetWagon, Wagon::COLLISION);
	
	PlanBase* tc = PlanBase::instance();
	
	tc->base()->p()[0] = robot_pos[0];
	tc->base()->p()[1] = robot_pos[1];
	tc->base()->R() = rotFromRpy(0,0,robot_pos[2]);
	
	tc->calcForwardKinematics();
	tc->flush();
	
	vector<double> control_pos;
	control_pos.resize(2);
	control_pos[0] = robot_pos[0] + ( rel_control_pos[0]*cos(robot_pos[2]) - rel_control_pos[1]*sin(robot_pos[2]));
	control_pos[1] = robot_pos[1] + ( rel_control_pos[0]*sin(robot_pos[2]) + rel_control_pos[1]*cos(robot_pos[2]));
	vector<double> tempPos;
	tempPos.resize(3);
	switch(grasp_type){
		case 1:
			tempPos[0] = control_pos[0] + (wagon->size_LongSide_Length/2.0)*cos(robot_pos[2]+rel_th);
			tempPos[1] = control_pos[1] + (wagon->size_LongSide_Length/2.0)*sin(robot_pos[2]+rel_th);
			tempPos[2] = robot_pos[2] + M_PI + rel_th;
			break;
	
		case 2:
			tempPos[0] = control_pos[0] + (wagon->size_ShortSide_Length/2.0)*cos(robot_pos[2]+rel_th);
			tempPos[1] = control_pos[1] + (wagon->size_ShortSide_Length/2.0)*sin(robot_pos[2]+rel_th);
			tempPos[2] = robot_pos[2] + M_PI/2.0 + rel_th;
			break;
	
		case 3:
			tempPos[0] = control_pos[0] + (wagon->size_LongSide_Length/2.0)*cos(robot_pos[2]+rel_th);
			tempPos[1] = control_pos[1] + (wagon->size_LongSide_Length/2.0)*sin(robot_pos[2]+rel_th);
			tempPos[2] = robot_pos[2] + rel_th;
			break;
	
		case 4:
			tempPos[0] = control_pos[0] + (wagon->size_ShortSide_Length/2.0)*cos(robot_pos[2]+rel_th);
			tempPos[1] = control_pos[1] + (wagon->size_ShortSide_Length/2.0)*sin(robot_pos[2]+rel_th);
			tempPos[2] = robot_pos[2] - M_PI/2.0 + rel_th;
			break;
	}
	wl->p()(0) = tempPos[0];
	wl->p()(1) = tempPos[1];
	wl->R() = rotFromRpy(0,0,tempPos[2]);
	
	tc->calcForwardKinematics();
	
	vector<double> out_control_point;
	double out_palm_th, out_rel_th;
	calcWagonGraspPose(robot_pos, out_palm_th, out_control_point, out_rel_th, out_pose);
	
	tc->flush();
	wp->changeWagonType(wp->targetWagon, Wagon::WagonType(wagon_type));
	return true;
};

bool WagonPushPlanner::calcPrePose(vector<double> robot_pos, Vector3 palm_pos, double palm_th, char LR, bool use_waist, Vector3 offset, vector<double>& out_pose){
	out_pose.clear();
	
	switch(LR){
		case 'L':
			break;
		case 'R':
			offset[1] = -offset[1];
			break;
	}
	
	robot = PlanBase::instance()->targetArmFinger;
	rl = PlanBase::instance()->base();
	
	wagon = WagonPlanner::instance()->targetWagon;
	wl = WagonPlanner::instance()->wagon();
	
	WagonPlanner* wp = WagonPlanner::instance();
	int wagon_type = wp->targetWagon->wagon_type;
	wp->changeWagonType(wp->targetWagon, Wagon::COLLISION);
	
	PlanBase* tc = PlanBase::instance();
	
	tc->base()->p()[0] = robot_pos[0];
	tc->base()->p()[1] = robot_pos[1];
	tc->base()->R() = rotFromRpy(0,0,robot_pos[2]);
	
	tc->calcForwardKinematics();
	tc->flush();
	
	Vector3 pre_palm_pos;
	pre_palm_pos[0] = palm_pos[0] + (offset[0]*cos(palm_th) - offset[1]*sin(palm_th));
	pre_palm_pos[1] = palm_pos[1] + (offset[0]*sin(palm_th) + offset[1]*cos(palm_th));
	pre_palm_pos[2] = palm_pos[2] + offset[2];
	
	Matrix3	yR, zR, palmR;
	yR(0, 0) = 0.0; yR(0, 1) = 0.0; yR(0, 2) = -1.0;
	yR(1, 0) = 0.0; yR(1, 1) = 1.0; yR(1, 2) = 0.0;
	yR(2, 0) = 1.0; yR(2, 1) = 0.0; yR(2, 2) = 0.0;
	
	zR(0, 0) = cos(palm_th); zR(0, 1) = -sin(palm_th); zR(0, 2) = 0.0;
	zR(1, 0) = sin(palm_th); zR(1, 1) =  cos(palm_th); zR(1, 2) = 0.0;
	zR(2, 0) = 0.0; zR(2, 1) = 0.0; zR(2, 2) = 1.0;
	
	palmR = zR * yR;
	
	switch(LR){
		case 'L':
			if(use_waist)
				tc->arm(1)->IK_arm(pre_palm_pos, palmR);
			else
				tc->arm(3)->IK_arm(pre_palm_pos, palmR);
			break;
		case 'R':
			if(use_waist)
				tc->arm(0)->IK_arm(pre_palm_pos, palmR);
			else
				tc->arm(2)->IK_arm(pre_palm_pos, palmR);
			break;
	}
	
	tc->calcForwardKinematics();
	tc->flush();
	//~ sleep(1);
	
	if(wp->isColliding()){
		return false;
	}
		
	for(int i=0;i<tc->bodyItemRobot()->body()->numJoints();i++){
		out_pose.push_back(tc->body()->joint(i)->q());
	}
	
	wp->changeWagonType(wp->targetWagon, Wagon::WagonType(wagon_type));
	
	return true;
}

void WagonPushPlanner::WagonGraspPlan(WagonMotionState preMotionState, vector<WagonMotionState>& outWagonGraspMotionSeq){
	PlanBase* tc = PlanBase::instance();
	WagonPlanner* wp = WagonPlanner::instance();
	outWagonGraspMotionSeq.clear();
	
	robot = PlanBase::instance()->targetArmFinger;
	rl = PlanBase::instance()->base();
	
	wagon = WagonPlanner::instance()->targetWagon;
	wl = WagonPlanner::instance()->wagon();
	
	vector<double>	robot_pos, out_pose, pre_pose;
	robot_pos.resize(3);
	robot_pos[0] = rl->p()(0);
	robot_pos[1] = rl->p()(1);
	robot_pos[2] = rpyFromRot(rl->R())(2);
	
	vector<double> out_control_point;
	double out_palm_th = 0.0, out_rel_th = 0.0;
	wp->changeWagonType(wp->targetWagon, Wagon::COLLISION);
	outWagonGraspMotionSeq.push_back(wp->getMotionState());
	
	calcWagonGraspPose(robot_pos, out_palm_th, out_control_point, out_rel_th, out_pose);
	Vector3 offset;
	offset[0] = -0.1;
	offset[1] = 0.0;
	offset[2] = 0.0;
	calcPrePose(robot_pos, tc->palm(2)->p(), out_palm_th, 'R', false, offset, pre_pose);
	calcPrePose(robot_pos, tc->palm(3)->p(), out_palm_th, 'L', false, offset, pre_pose);
	outWagonGraspMotionSeq.push_back(wp->getMotionState());
	
	calcWagonGraspPose(robot_pos, out_palm_th, out_control_point, out_rel_th, out_pose);
	wp->changeWagonType(wp->targetWagon, Wagon::PUSHING);
	outWagonGraspMotionSeq.push_back(wp->getMotionState());
	
	//~ out_control_point[0] = wp->targetWagon->controlDist;
	//~ out_control_point[1] = 0.0;
	//~ calcRelativeWagonGraspPose(robot_pos, wp->targetWagon->state, out_control_point, out_rel_th, out_pose);
	//~ outWagonGraspMotionSeq.push_back(wp->getMotionState());
	//~ 
	//~ calcStandardWagonGraspPose(robot_pos, WagonPlanner::instance()->targetWagon->state, out_pose);
	//~ outWagonGraspMotionSeq.push_back(wp->getMotionState());
	//~ 
	//~ os<<endl;
}

void WagonPushPlanner::WagonReleasePlan(WagonMotionState preMotionState, vector<WagonMotionState>& outWagonGraspMotionSeq){
	PlanBase* tc = PlanBase::instance();
	WagonPlanner* wp = WagonPlanner::instance();
	outWagonGraspMotionSeq.clear();
	
	robot = PlanBase::instance()->targetArmFinger;
	rl = PlanBase::instance()->base();
	
	wagon = WagonPlanner::instance()->targetWagon;
	wl = WagonPlanner::instance()->wagon();
	
	bool isWagonGrasp = false;
	wp->setMotionState(preMotionState);
	for(unsigned int i=0;i<tc->interObjectList.size();i++){
		if(wp->targetWagon->name()==tc->interObjectList[i].slaveItem->name()){
			isWagonGrasp = (wp->targetWagon->wagon_type==Wagon::PUSHING);
			//~ wp->changeWagonType(wp->targetWagon, Wagon::PUSHING);
		}
	}
	
	if(!isWagonGrasp){
		os<<"Wagon Release Error : Wagon is not Grasped"<<endl;
		return;
	}
	
	wp->changeWagonType(wp->targetWagon, Wagon::COLLISION);
	
	vector<double>	robot_pos, out_pose, pre_pose;
	robot_pos.resize(3);
	robot_pos[0] = rl->p()(0);
	robot_pos[1] = rl->p()(1);
	robot_pos[2] = rpyFromRot(rl->R())(2);
	
	vector<double> out_control_point;
	double out_palm_th = 0.0, out_rel_th = 0.0;
	calcWagonGraspPose(robot_pos, out_palm_th, out_control_point, out_rel_th, out_pose);
	outWagonGraspMotionSeq.push_back(wp->getMotionState());
	
	Vector3 offset;
	offset[0] = -0.1;
	offset[1] = 0.0;
	offset[2] = 0.0;	
	calcPrePose(robot_pos, tc->palm(2)->p(), out_palm_th, 'R', false, offset, pre_pose);
	calcPrePose(robot_pos, tc->palm(3)->p(), out_palm_th, 'L', false, offset, pre_pose);
	outWagonGraspMotionSeq.push_back(wp->getMotionState());
	
	tc->bodyItemRobot()->setPresetPose(BodyItem::STANDARD_POSE);
	outWagonGraspMotionSeq.push_back(wp->getMotionState());
}

bool WagonPushPlanner::calcWagonPosOnVoronoi(vector<vector<CollisionMapData> >& Map, vector<double>& robot_pos, int wagon_state, double control_dist, double& rel_th, bool change_rel, double threshold){
	double limit_angle = deg2rad(30.0);
	
	if(Map.empty()){
		os<<"Error : Map is empty"<<endl;
		return false;
	}
	
	WagonPlanner* wp = WagonPlanner::instance();
	VoronoiPathPlanner* vp = VoronoiPathPlanner::instance();
	
	double incre_th = deg2rad(0.1);
	vector<double> temp_r_pos, temp_w_pos, temp_control_pos;
	temp_r_pos.resize(3);
	temp_r_pos = robot_pos;
	temp_w_pos.resize(2);
	temp_control_pos.resize(2);
	
	vector<int> i_temp_w_pos, i_temp_control_pos;
	i_temp_w_pos.resize(2);
	i_temp_control_pos.resize(2);
	
	double wagon_l = 0.0, temp_rel_th = rel_th;
	
	switch(wagon_state){
		case 1:
			wagon_l = wagon->size_LongSide_Length;
			break;
			
		case 2:
			wagon_l = wagon->size_ShortSide_Length;
			break;
	
		case 3:
			wagon_l = wagon->size_LongSide_Length;
			break;
			
		case 4:
			wagon_l = wagon->size_ShortSide_Length;
			break;
	}
	
	double temp_dist;
	double next_dist;
	double p_rot, p_dist, m_rot, m_dist;
	bool p_flg = false, m_flg = false;
	
	if(!change_rel){
		
		temp_control_pos[0] = temp_r_pos[0] + (control_dist+(wagon_l/2.0))*cos(temp_r_pos[2]);
		temp_control_pos[1] = temp_r_pos[1] + (control_dist+(wagon_l/2.0))*sin(temp_r_pos[2]);
		i_temp_control_pos[0] = (int)round( (temp_control_pos[0] - vp->x_llimit) / vp->cell_size );
		i_temp_control_pos[1] = (int)round( (temp_control_pos[1] - vp->y_llimit) / vp->cell_size );
		temp_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
		next_dist = temp_dist;
		p_flg = false;
		m_flg = false;
		
		if((!Map[i_temp_control_pos[0]][i_temp_control_pos[1]].collision)&&(temp_dist>threshold)){
			while( next_dist <= temp_dist ){
				if(next_dist <= threshold){
					p_flg = true;
					break;
				}
				temp_r_pos[2] += incre_th;
				temp_control_pos[0] = temp_r_pos[0] + (control_dist+(wagon_l/2.0))*cos(temp_r_pos[2]);
				temp_control_pos[1] = temp_r_pos[1] + (control_dist+(wagon_l/2.0))*sin(temp_r_pos[2]);
				
				i_temp_control_pos[0] = (int)round( (temp_control_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_control_pos[1] = (int)round( (temp_control_pos[1] - vp->y_llimit) / vp->cell_size );
				next_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
			}
			if(!p_flg){
				temp_r_pos[2] -= incre_th;
				temp_control_pos[0] = temp_r_pos[0] + (control_dist+(wagon_l/2.0))*cos(temp_r_pos[2]);
				temp_control_pos[1] = temp_r_pos[1] + (control_dist+(wagon_l/2.0))*sin(temp_r_pos[2]);
				
				i_temp_control_pos[0] = (int)round( (temp_control_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_control_pos[1] = (int)round( (temp_control_pos[1] - vp->y_llimit) / vp->cell_size );
				next_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
			}
			p_dist = next_dist;
			p_rot = temp_r_pos[2] - robot_pos[2];
			if(next_dist == temp_dist)
				p_rot = 0.0;
			
				
			next_dist = temp_dist;
			temp_r_pos[2] = robot_pos[2];
			
			while( next_dist <= temp_dist ){
				if(next_dist <= threshold){
					m_flg = true;
					break;
				}
				temp_r_pos[2] -= incre_th;
				temp_control_pos[0] = temp_r_pos[0] + (control_dist+(wagon_l/2.0))*cos(temp_r_pos[2]);
				temp_control_pos[1] = temp_r_pos[1] + (control_dist+(wagon_l/2.0))*sin(temp_r_pos[2]);
				
				i_temp_control_pos[0] = (int)round( (temp_control_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_control_pos[1] = (int)round( (temp_control_pos[1] - vp->y_llimit) / vp->cell_size );
				next_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
			}
			if(!m_flg){
				temp_r_pos[2] += incre_th;
				temp_control_pos[0] = temp_r_pos[0] + (control_dist+(wagon_l/2.0))*cos(temp_r_pos[2]);
				temp_control_pos[1] = temp_r_pos[1] + (control_dist+(wagon_l/2.0))*sin(temp_r_pos[2]);
				
				i_temp_control_pos[0] = (int)round( (temp_control_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_control_pos[1] = (int)round( (temp_control_pos[1] - vp->y_llimit) / vp->cell_size );
				next_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
			}
			m_dist = next_dist;
			m_rot = temp_r_pos[2] - robot_pos[2];
			if(next_dist == temp_dist)
				m_rot = 0.0;
			
			if(p_dist==m_dist){
				if(fabs(p_rot)<=fabs(m_rot))
					robot_pos[2] += p_rot;
				else
					robot_pos[2] += m_rot;
			}
			else if(p_dist<=m_dist)
				robot_pos[2] += p_rot;
			else
				robot_pos[2] += m_rot;
		}
		else{
			while(Map[i_temp_control_pos[0]][i_temp_control_pos[1]].collision){
				temp_r_pos[2] += incre_th;
				temp_control_pos[0] = temp_r_pos[0] + (control_dist+(wagon_l/2.0))*cos(temp_r_pos[2]);
				temp_control_pos[1] = temp_r_pos[1] + (control_dist+(wagon_l/2.0))*sin(temp_r_pos[2]);
				
				i_temp_control_pos[0] = (int)round( (temp_control_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_control_pos[1] = (int)round( (temp_control_pos[1] - vp->y_llimit) / vp->cell_size );
			}
			p_rot = temp_r_pos[2] - robot_pos[2];
			
			temp_r_pos[2] = robot_pos[2];
			temp_control_pos[0] = temp_r_pos[0] + (control_dist+(wagon_l/2.0))*cos(temp_r_pos[2]);
			temp_control_pos[1] = temp_r_pos[1] + (control_dist+(wagon_l/2.0))*sin(temp_r_pos[2]);
			
			i_temp_control_pos[0] = (int)round( (temp_control_pos[0] - vp->x_llimit) / vp->cell_size );
			i_temp_control_pos[1] = (int)round( (temp_control_pos[1] - vp->y_llimit) / vp->cell_size );
			
			while(Map[i_temp_control_pos[0]][i_temp_control_pos[1]].collision){
				temp_r_pos[2] -= incre_th;
				temp_control_pos[0] = temp_r_pos[0] + (control_dist+(wagon_l/2.0))*cos(temp_r_pos[2]);
				temp_control_pos[1] = temp_r_pos[1] + (control_dist+(wagon_l/2.0))*sin(temp_r_pos[2]);
				
				i_temp_control_pos[0] = (int)round( (temp_control_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_control_pos[1] = (int)round( (temp_control_pos[1] - vp->y_llimit) / vp->cell_size );
			}
			m_rot = temp_r_pos[2] - robot_pos[2];
			
			if(fabs(p_rot)<=fabs(m_rot))
				robot_pos[2] += p_rot;
			else
				robot_pos[2] += m_rot;
		}
	}
	
	if(change_rel){
		
		temp_control_pos[0] = temp_r_pos[0] + control_dist*cos(temp_r_pos[2]);
		temp_control_pos[1] = temp_r_pos[1] + control_dist*sin(temp_r_pos[2]);
		i_temp_control_pos[0] = (int)round( (temp_control_pos[0] - vp->x_llimit) / vp->cell_size );
		i_temp_control_pos[1] = (int)round( (temp_control_pos[1] - vp->y_llimit) / vp->cell_size );
		temp_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
		next_dist = temp_dist;
		p_flg = false;
		m_flg = false;
		
		if((!Map[i_temp_control_pos[0]][i_temp_control_pos[1]].collision)&&(temp_dist>threshold)){
			while( next_dist <= temp_dist ){
				if(next_dist <= threshold){
					p_flg = true;
					break;
				}
				temp_r_pos[2] += incre_th;
				temp_control_pos[0] = temp_r_pos[0] + control_dist*cos(temp_r_pos[2]);
				temp_control_pos[1] = temp_r_pos[1] + control_dist*sin(temp_r_pos[2]);
				
				i_temp_control_pos[0] = (int)round( (temp_control_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_control_pos[1] = (int)round( (temp_control_pos[1] - vp->y_llimit) / vp->cell_size );
				next_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
			}
			if(!p_flg){
				temp_r_pos[2] -= incre_th;
				temp_control_pos[0] = temp_r_pos[0] + control_dist*cos(temp_r_pos[2]);
				temp_control_pos[1] = temp_r_pos[1] + control_dist*sin(temp_r_pos[2]);
				
				i_temp_control_pos[0] = (int)round( (temp_control_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_control_pos[1] = (int)round( (temp_control_pos[1] - vp->y_llimit) / vp->cell_size );
				next_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
			}
			p_dist = next_dist;
			p_rot = temp_r_pos[2] - robot_pos[2];
			if(next_dist == temp_dist)
				p_rot = 0.0;
				
			next_dist = temp_dist;
			temp_r_pos[2] = robot_pos[2];
			while( next_dist <= temp_dist ){
				if(next_dist <= threshold){
					m_flg = true;
					break;
				}
				temp_r_pos[2] -= incre_th;
				temp_control_pos[0] = temp_r_pos[0] + control_dist*cos(temp_r_pos[2]);
				temp_control_pos[1] = temp_r_pos[1] + control_dist*sin(temp_r_pos[2]);
				
				i_temp_control_pos[0] = (int)round( (temp_control_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_control_pos[1] = (int)round( (temp_control_pos[1] - vp->y_llimit) / vp->cell_size );
				next_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
			}
			if(!m_flg){
				temp_r_pos[2] += incre_th;
				temp_control_pos[0] = temp_r_pos[0] + control_dist*cos(temp_r_pos[2]);
				temp_control_pos[1] = temp_r_pos[1] + control_dist*sin(temp_r_pos[2]);
				
				i_temp_control_pos[0] = (int)round( (temp_control_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_control_pos[1] = (int)round( (temp_control_pos[1] - vp->y_llimit) / vp->cell_size );
				next_dist = Map[i_temp_control_pos[0]][i_temp_control_pos[1]].dist_from_voronoi;
			}
			m_dist = next_dist;
			m_rot = temp_r_pos[2] - robot_pos[2];
			if(next_dist == temp_dist)
				m_rot = 0.0;
			
			if(p_dist==m_dist){
				if(fabs(p_rot)<=fabs(m_rot))
					robot_pos[2] += p_rot;
				else
					robot_pos[2] += m_rot;
			}
			else if(p_dist<=m_dist)
				robot_pos[2] += p_rot;
			else
				robot_pos[2] += m_rot;
		}
		else{
			while(Map[i_temp_control_pos[0]][i_temp_control_pos[1]].collision){
				temp_r_pos[2] += incre_th;
				temp_control_pos[0] = temp_r_pos[0] + control_dist*cos(temp_r_pos[2]);
				temp_control_pos[1] = temp_r_pos[1] + control_dist*sin(temp_r_pos[2]);
				
				i_temp_control_pos[0] = (int)round( (temp_control_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_control_pos[1] = (int)round( (temp_control_pos[1] - vp->y_llimit) / vp->cell_size );
			}
			p_rot = temp_r_pos[2] - robot_pos[2];
			
			temp_r_pos[2] = robot_pos[2];
			temp_control_pos[0] = temp_r_pos[0] + control_dist*cos(temp_r_pos[2]);
			temp_control_pos[1] = temp_r_pos[1] + control_dist*sin(temp_r_pos[2]);
			
			i_temp_control_pos[0] = (int)round( (temp_control_pos[0] - vp->x_llimit) / vp->cell_size );
			i_temp_control_pos[1] = (int)round( (temp_control_pos[1] - vp->y_llimit) / vp->cell_size );
			while(Map[i_temp_control_pos[0]][i_temp_control_pos[1]].collision){
				temp_r_pos[2] -= incre_th;
				temp_control_pos[0] = temp_r_pos[0] + control_dist*cos(temp_r_pos[2]);
				temp_control_pos[1] = temp_r_pos[1] + control_dist*sin(temp_r_pos[2]);
				
				i_temp_control_pos[0] = (int)round( (temp_control_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_control_pos[1] = (int)round( (temp_control_pos[1] - vp->y_llimit) / vp->cell_size );
			}
			m_rot = temp_r_pos[2] - robot_pos[2];
			
			if(fabs(p_rot)<=fabs(m_rot))
				robot_pos[2] += p_rot;
			else
				robot_pos[2] += m_rot;
		}
	
		//rel_th
		temp_control_pos[0] = robot_pos[0] + control_dist*cos(robot_pos[2]);
		temp_control_pos[1] = robot_pos[1] + control_dist*sin(robot_pos[2]);
		
		temp_w_pos[0] = temp_control_pos[0] + wagon_l*cos(robot_pos[2]+rel_th);
		temp_w_pos[1] = temp_control_pos[1] + wagon_l*sin(robot_pos[2]+rel_th);
		i_temp_w_pos[0] = (int)round( (temp_w_pos[0] - vp->x_llimit) / vp->cell_size );
		i_temp_w_pos[1] = (int)round( (temp_w_pos[1] - vp->y_llimit) / vp->cell_size );
		
		temp_dist = Map[i_temp_w_pos[0]][i_temp_w_pos[1]].dist_from_voronoi;
		next_dist = temp_dist;
		p_flg = false;
		m_flg = false;
	
		if((!Map[i_temp_w_pos[0]][i_temp_w_pos[1]].collision)&&(temp_dist>threshold)){
			while( next_dist <= temp_dist ){
				if(next_dist <= threshold){
					p_flg = true;
					break;
				}
				if(temp_rel_th > limit_angle)
					break;
				temp_rel_th += incre_th;
				temp_w_pos[0] = temp_control_pos[0] + wagon_l*cos(robot_pos[2]+temp_rel_th);
				temp_w_pos[1] = temp_control_pos[1] + wagon_l*sin(robot_pos[2]+temp_rel_th);
				i_temp_w_pos[0] = (int)round( (temp_w_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_w_pos[1] = (int)round( (temp_w_pos[1] - vp->y_llimit) / vp->cell_size );
				next_dist = Map[i_temp_w_pos[0]][i_temp_w_pos[1]].dist_from_voronoi;
			}
			if(!p_flg){
				temp_rel_th -= incre_th;
				temp_w_pos[0] = temp_control_pos[0] + wagon_l*cos(robot_pos[2]+temp_rel_th);
				temp_w_pos[1] = temp_control_pos[1] + wagon_l*sin(robot_pos[2]+temp_rel_th);
				i_temp_w_pos[0] = (int)round( (temp_w_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_w_pos[1] = (int)round( (temp_w_pos[1] - vp->y_llimit) / vp->cell_size );
				next_dist = Map[i_temp_w_pos[0]][i_temp_w_pos[1]].dist_from_voronoi;
			}
			p_dist = next_dist;
			p_rot = temp_rel_th - rel_th;
			if(next_dist == temp_dist)
				p_rot = 0.0;
				
			next_dist = temp_dist;
			temp_rel_th = rel_th;
			while( next_dist <= temp_dist ){
				if(next_dist <= threshold){
					m_flg = true;
					break;
				}
				if(temp_rel_th < -limit_angle)
					break;
				temp_rel_th -= incre_th;
				temp_w_pos[0] = temp_control_pos[0] + wagon_l*cos(robot_pos[2]+temp_rel_th);
				temp_w_pos[1] = temp_control_pos[1] + wagon_l*sin(robot_pos[2]+temp_rel_th);
				i_temp_w_pos[0] = (int)round( (temp_w_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_w_pos[1] = (int)round( (temp_w_pos[1] - vp->y_llimit) / vp->cell_size );
				next_dist = Map[i_temp_w_pos[0]][i_temp_w_pos[1]].dist_from_voronoi;
			}
			if(!m_flg){
				temp_rel_th += incre_th;
				temp_w_pos[0] = temp_control_pos[0] + wagon_l*cos(robot_pos[2]+temp_rel_th);
				temp_w_pos[1] = temp_control_pos[1] + wagon_l*sin(robot_pos[2]+temp_rel_th);
				i_temp_w_pos[0] = (int)round( (temp_w_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_w_pos[1] = (int)round( (temp_w_pos[1] - vp->y_llimit) / vp->cell_size );
				next_dist = Map[i_temp_w_pos[0]][i_temp_w_pos[1]].dist_from_voronoi;
			}
			m_dist = next_dist;
			m_rot = temp_rel_th - rel_th;
			if(next_dist == temp_dist)
				m_rot = 0.0;
			
			if(p_dist==m_dist){
				if(fabs(p_rot)<=fabs(m_rot))
					rel_th += p_rot;
				else
					rel_th += m_rot;
			}
			else if(p_dist<=m_dist)
				rel_th += p_rot;
			else
				rel_th += m_rot;
		}
		else{
			while(Map[i_temp_w_pos[0]][i_temp_w_pos[1]].collision){
				if(temp_rel_th > limit_angle)
					break;
				temp_rel_th += incre_th;
				temp_w_pos[0] = temp_control_pos[0] + wagon_l*cos(robot_pos[2]+temp_rel_th);
				temp_w_pos[1] = temp_control_pos[1] + wagon_l*sin(robot_pos[2]+temp_rel_th);
				i_temp_w_pos[0] = (int)round( (temp_w_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_w_pos[1] = (int)round( (temp_w_pos[1] - vp->y_llimit) / vp->cell_size );
			}
			p_rot = temp_rel_th - rel_th;
			
			temp_rel_th = rel_th;
			temp_w_pos[0] = temp_control_pos[0] + wagon_l*cos(robot_pos[2]+temp_rel_th);
			temp_w_pos[1] = temp_control_pos[1] + wagon_l*sin(robot_pos[2]+temp_rel_th);
			i_temp_w_pos[0] = (int)round( (temp_w_pos[0] - vp->x_llimit) / vp->cell_size );
			i_temp_w_pos[1] = (int)round( (temp_w_pos[1] - vp->y_llimit) / vp->cell_size );
			while(Map[i_temp_w_pos[0]][i_temp_w_pos[1]].collision){
				if(temp_rel_th < -limit_angle)
					break;
				temp_rel_th -= incre_th;
				temp_w_pos[0] = temp_control_pos[0] + wagon_l*cos(robot_pos[2]+temp_rel_th);
				temp_w_pos[1] = temp_control_pos[1] + wagon_l*sin(robot_pos[2]+temp_rel_th);
				i_temp_w_pos[0] = (int)round( (temp_w_pos[0] - vp->x_llimit) / vp->cell_size );
				i_temp_w_pos[1] = (int)round( (temp_w_pos[1] - vp->y_llimit) / vp->cell_size );
			}
			m_rot = temp_rel_th - rel_th;
			
			if(fabs(p_rot)<=fabs(m_rot))
				rel_th += p_rot;
			else
				rel_th += m_rot;
		}
		
	}
	
	return true;
}

bool WagonPushPlanner::calcWagonPushPath(vector<vector<CollisionMapData> >& Map, vector<double> start, vector<double> goal, vector<vector<double> >& out_path){
	out_path.clear();
	
	PlanBase* tc = PlanBase::instance();
	WagonPlanner* wp = WagonPlanner::instance();
	
	vector<double>	robot_pos, out_pose, pre_pose;
	robot_pos.resize(3);
	robot_pos[0] = start[0];
	robot_pos[1] = start[1];
	robot_pos[2] = start[2];
	
	vector<double> out_control_point;
	double out_palm_th = 0.0, out_rel_th = 0.0;
	wp->changeWagonType(wp->targetWagon, Wagon::COLLISION);
	
	if(!calcWagonGraspPose(robot_pos, out_palm_th, out_control_point, out_rel_th, out_pose)){
		os<<"Error : robot can't grasp wagon"<<endl;
		return false;
	}
	Vector3 offset;
	offset[0] = -0.1;
	offset[1] = 0.0;
	offset[2] = 0.0;
	if(!calcPrePose(robot_pos, tc->palm(2)->p(), out_palm_th, 'R', false, offset, pre_pose)){
		os<<"Error : pre grasp pose can't found"<<endl;
		return false;
	}
	if(!calcPrePose(robot_pos, tc->palm(3)->p(), out_palm_th, 'L', false, offset, pre_pose)){
		os<<"Error : pre grasp pose can't found"<<endl;
		return false;
	}
	
	if(!calcWagonGraspPose(robot_pos, out_palm_th, out_control_point, out_rel_th, out_pose)){
		os<<"Error : robot can't grasp wagon"<<endl;
		return false;
	}
	
	out_control_point[0] = wp->targetWagon->controlDist;
	out_control_point[1] = 0.0;
	calcRelativeWagonGraspPose(robot_pos, wp->targetWagon->state, out_control_point, out_rel_th, out_pose);
	
	calcStandardWagonGraspPose(robot_pos, WagonPlanner::instance()->targetWagon->state, out_pose);
	wp->changeWagonType(wp->targetWagon, Wagon::PUSHING);
	
	vector<double> rob_pos, arm_pose;
	rob_pos.resize(3);
	VoronoiPathPlanner::instance()->planVoronoiPath(Map, start, goal, out_path);
	rob_pos[0] = out_path[0][0];
	rob_pos[1] = out_path[0][1];
	rob_pos[2] = out_path[0][2];
	for(unsigned int i=1;i<out_path.size();i++){
		//~ tc->body()->link(0)->p()(0) = out_path[i][0];
		//~ tc->body()->link(0)->p()(1) = out_path[i][1];
		//~ tc->body()->link(0)->R = rotFromRpy(0.0,0.0,out_path[i][2]);
		//~ tc->calcForwardKinematics();
		//~ wp->flush();
		
		if((rob_pos[0]==out_path[i][0])&&(rob_pos[1]==out_path[i][1]))
			continue;
		rob_pos[0] = out_path[i][0];
		rob_pos[1] = out_path[i][1];
		calcWagonPosOnVoronoi(Map, rob_pos, wp->targetWagon->state, wp->targetWagon->controlDist, wp->targetWagon->rel_th, true, 0.2);
		tc->body()->link(0)->p()(0) = rob_pos[0];
		tc->body()->link(0)->p()(1) = rob_pos[1];
		tc->body()->link(0)->R() = rotFromRpy(0.0,0.0,rob_pos[2]);
		
		tc->calcForwardKinematics();
		//~ cout<<rob_pos[0]<<"	"<<rob_pos[1]<<"	"<<rad2deg(rob_pos[2])<<"	"<<wp->targetWagon->rel_th<<"	"<<wp->targetWagon->state<<"	"<<wp->targetWagon->controlDist<<endl;
		calcRelativeWagonGraspPose(rob_pos, wp->targetWagon->state, out_control_point, wp->targetWagon->rel_th, arm_pose);
		tc->flush();
	}
	
	return true;
}
