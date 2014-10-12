#include <cnoid_planner_controller.h>

using namespace std;
using namespace cnoid;
using namespace grasp;

const double open_gripper_angle = -40.0, close_gripper_angle = 0.0;

RPS_Controller::RPS_Controller()  : os (MessageView::mainInstance()->cout() ){
	pb = PlanBase::instance();
	wp = WagonPlanner::instance();
	wpp = WagonPushPlanner::instance();
	hp = HumanPlanner::instance();
}

RPS_Controller::~RPS_Controller() {
}

RPS_Controller* RPS_Controller::instance(RPS_Controller *gc) {
	static RPS_Controller* instance = (gc) ? gc : new RPS_Controller();
	if(gc) instance = gc;
	return instance;
}

bool RPS_Controller::SetRobotPose(vector<double> pos, vector<double> rpy, vector<double> joint_angle){
	if((!pb->targetArmFinger)||(pos.size()!=3)||(rpy.size()!=3))
		return false;
		
	for(unsigned int i=0;i<3;i++)
		pb->body()->link(0)->p()(i) = pos[i];
	
	pb->body()->link(0)->R() = rotFromRpy(rpy[0], rpy[1], rpy[2]);
	
	for(int j=0;j<pb->body()->numJoints();j++){
		if(j == joint_angle.size())
			break;
		pb->body()->joint(j)->q() = joint_angle[j];
	}
	
	pb->calcForwardKinematics();
	pb->flush();
		
	return true;
}

bool RPS_Controller::SetWagonPose(vector<double> pos, vector<double> rpy){
	if((!wp->targetWagon)||(pos.size()!=3)||(rpy.size()!=3))
		return false;
		
	for(unsigned int i=0;i<3;i++)
		wp->wagon()->p()(i) = pos[i];
	
	wp->wagon()->R() = rotFromRpy(rpy[0], rpy[1], rpy[2]);
	
	pb->calcForwardKinematics();
	pb->flush();
		
	return true;
}

bool RPS_Controller::SetObjectPose(vector<double> pos, vector<double> rpy){
	if((!pb->targetObject)||(pos.size()!=3)||(rpy.size()!=3))
		return false;
		
	for(unsigned int i=0;i<3;i++)
		pb->object()->p()(i) = pos[i];
	
	pb->object()->R() = rotFromRpy(rpy[0], rpy[1], rpy[2]);
	
	pb->calcForwardKinematics();
	pb->flush();
		
	return true;
}

bool RPS_Controller::SetHumanPose(vector<double> pos, vector<double> rpy, vector<double> joint_angle){
	if((!hp->targetHuman)||(pos.size()!=3)||(rpy.size()!=3))
		return false;
		
	for(unsigned int i=0;i<3;i++)
		hp->body()->link(0)->p()(i) = pos[i];
	
	hp->body()->link(0)->R() = rotFromRpy(rpy[0], rpy[1], rpy[2]);
	
	hp->setHumanPose(joint_angle);
	
	hp->flush();
		
	return true;
}

bool RPS_Controller::GraspObjPlan(vector<vector<double> >& out_joint_angle){
	out_joint_angle.clear();
	
	double out_palm_th;
	vector<double> robot_pos(3), pre_pose, grasp_pose, out_pose;
	robot_pos[0] = pb->body()->link(0)->p()(0);
	robot_pos[1] = pb->body()->link(0)->p()(1);
	robot_pos[2] = rpyFromRot(pb->body()->link(0)->R())(2);
	grasp_pose.clear();
	
	bool init = PlanBase::instance()->initial();
	if(!init){
		cout << "Failed: Grasp Planning Initial" << endl;
		return false;
	}
	bool result = GraspController::instance()->loadAndSelectGraspPattern();
	
	if(result){
		for(int i=0;i<pb->bodyItemRobot()->body()->numJoints();i++){
			grasp_pose.push_back(pb->body()->joint(i)->q());
		}
		out_palm_th = atan2( pb->object()->p()(1) - pb->fingers(0,0)->joint(0)->p()(1), pb->object()->p()(0) - pb->fingers(0,0)->joint(0)->p()(0) );
		
		Vector3 offset, palm_p = pb->palm(0)->p();
		offset[0] = -0.15;
		offset[1] = 0.0;
		offset[2] = 0.0;
		result = wpp->calcPrePose(robot_pos, palm_p, out_palm_th, 'R', false, offset, pre_pose);
		
		offset[0] = 0.0;
		offset[1] = 0.0;
		offset[2] = 0.05;
		result = wpp->calcPrePose(robot_pos, palm_p, out_palm_th, 'R', false, offset, out_pose);
	}
	
	out_joint_angle.push_back(pre_pose);
	out_joint_angle.push_back(grasp_pose);
	out_joint_angle.push_back(out_pose);
	
	for(int j=0;j<grasp_pose.size();j++){
		pb->body()->joint(j)->q() = grasp_pose[j];
	}
	pb->calcForwardKinematics();
	pb->flush();
	/////////////////////////////
	
	return result;
}

bool RPS_Controller::calcGraspWagonPose(vector<double>& out_joint_angle){
	out_joint_angle.clear();
	
	double out_palm_th, rel_th;
	vector<double> robot_pos(3), rel_control_point_pos;
	robot_pos[0] = pb->body()->link(0)->p()(0);
	robot_pos[1] = pb->body()->link(0)->p()(1);
	robot_pos[2] = rpyFromRot(pb->body()->link(0)->R())(2);

	bool result = wpp->calcWagonGraspPose(robot_pos, out_palm_th, rel_control_point_pos, rel_th, out_joint_angle);
	
	return result;
}

bool RPS_Controller::GraspWagonPlan(int& grasp_type, vector<vector<double> >& out_joint_angle, double& out_palm_th, vector<double>& rel_control_point_pos, double& rel_th){
	grasp_type = 0;
	out_joint_angle.clear();
	rel_control_point_pos.clear();
	rel_th = 0.0;
	
	Vector3 r_palm_p, l_palm_p, offset;
	vector<double> robot_pos(3), pre_pose0, pre_pose1, grasp_pose, out_pose;
	robot_pos[0] = pb->body()->link(0)->p()(0);
	robot_pos[1] = pb->body()->link(0)->p()(1);
	robot_pos[2] = rpyFromRot(pb->body()->link(0)->R())(2);

	bool result = wpp->calcWagonGraspPose(robot_pos, out_palm_th, rel_control_point_pos, rel_th, grasp_pose);
	r_palm_p = pb->palm(2)->p();
	l_palm_p = pb->palm(3)->p();
	
	if(result){
		offset[0] = -0.12;
		offset[1] = 0.035;
		offset[2] = 0.0;
		result = wpp->calcPrePose(robot_pos, r_palm_p, out_palm_th, 'R', false, offset, pre_pose0);
		result = wpp->calcPrePose(robot_pos, l_palm_p, out_palm_th, 'L', false, offset, pre_pose0);
	}
	if(result){
		offset[0] = 0.0;
		offset[1] = 0.020;
		offset[2] = 0.0;
		result = wpp->calcPrePose(robot_pos, r_palm_p, out_palm_th, 'R', false, offset, pre_pose1);
		result = wpp->calcPrePose(robot_pos, l_palm_p, out_palm_th, 'L', false, offset, pre_pose1);
	}
	
	out_joint_angle.push_back(pre_pose0);
	out_joint_angle.push_back(pre_pose1);
	out_joint_angle.push_back(grasp_pose);
	
	wp->changeWagonType(wp->targetWagon, Wagon::PUSHING);
	
	rel_control_point_pos[0] = wp->targetWagon->controlDist;
	rel_control_point_pos[1] = 0.0;
	wpp->calcRelativeWagonGraspPose(robot_pos, wp->targetWagon->state, rel_control_point_pos, rel_th, out_pose);
	out_joint_angle.push_back(out_pose);
	wpp->calcStandardWagonGraspPose(robot_pos, wp->targetWagon->state, out_pose);
	out_joint_angle.push_back(out_pose);
	/////////////////////////////
	
	grasp_type = wp->targetWagon->state;
	
	return result;
}

bool RPS_Controller::ReleaseWagonPlan(vector<vector<double> >& out_joint_angle){
	bool result = false;
	
	vector<double> temp_joint_angle;
	temp_joint_angle.clear();
	
	Vector3 r_palm_p, l_palm_p, offset;
	vector<double> robot_pos(3), pre_pose0, pre_pose1, grasp_pose, out_pose, rel_control_point_pos;
	double palm_th = 0.0, rel_th = 0.0;
	robot_pos[0] = pb->body()->link(0)->p()(0);
	robot_pos[1] = pb->body()->link(0)->p()(1);
	robot_pos[2] = rpyFromRot(pb->body()->link(0)->R())(2);
	wpp->calcWagonGraspPose(robot_pos, palm_th, rel_control_point_pos, rel_th, grasp_pose);
	
	pb->fingers(0,0)->joint(0)->q() = deg2rad(open_gripper_angle);
	pb->fingers(1,0)->joint(0)->q() = deg2rad(open_gripper_angle);
	
	out_joint_angle.clear();
	for(int j=0;j<pb->body()->numJoints();j++){
		temp_joint_angle.push_back(pb->body()->joint(j)->q());
	}
	out_joint_angle.push_back(temp_joint_angle);
		
	r_palm_p = pb->palm(2)->p();
	l_palm_p = pb->palm(3)->p();

	offset[0] = 0.0;
	offset[1] = 0.02;
	offset[2] = 0.0;
	result = wpp->calcPrePose(robot_pos, r_palm_p, palm_th, 'R', false, offset, pre_pose0);
	result = wpp->calcPrePose(robot_pos, l_palm_p, palm_th, 'L', false, offset, pre_pose0);
	
	offset[0] = -0.12;
	offset[1] = 0.02;
	offset[2] = 0.0;
	result = wpp->calcPrePose(robot_pos, r_palm_p, palm_th, 'R', false, offset, pre_pose1);
	result = wpp->calcPrePose(robot_pos, l_palm_p, palm_th, 'L', false, offset, pre_pose1);
	
	out_joint_angle.push_back(pre_pose0);
	out_joint_angle.push_back(pre_pose1);
	
	pb->bodyItemRobot()->setPresetPose(BodyItem::STANDARD_POSE);
	temp_joint_angle.clear();	
	for(int j=0;j<pb->body()->numJoints();j++){
		temp_joint_angle.push_back(pb->body()->joint(j)->q());
	}
	out_joint_angle.push_back(temp_joint_angle);
	
	return result;
}
