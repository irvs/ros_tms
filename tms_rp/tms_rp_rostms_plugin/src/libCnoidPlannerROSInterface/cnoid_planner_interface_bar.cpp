#include <cnoid_planner_interface_bar.h>

#include <tms_msg_db/tmsdb_get_robots_info.h>
#include <tms_msg_db/tmsdb_get_movable_furnitures_info.h>
#include <tms_msg_db/tmsdb_get_person_behavior_info.h>
#include <tms_msg_db/tmsdb_get_person_info.h>

#include <tms_msg_rp/rps_map_full.h>
#include <tms_msg_rp/rps_cnoid_grasp_obj_planning.h>
#include <tms_msg_rp/rps_cnoid_grasp_wagon_planning.h>
#include <tms_msg_rp/rps_cnoid_PRM_planning.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <cnoid/MessageView>
#include <Grasp/PlanBase.h>
#include <Grasp/VectorMath.h>
#include <libWagonPlanner/WagonPlanner.h>
#include <libGiveObjToHumanPlanner/GiveObjToHumanPlanner.h>

#define rad2deg(x)	((x)*(180.0)/M_PI)
#define deg2rad(x)	((x)*M_PI/180.0)

int argc;
char **argv;

//------------------------------------------------------------------------------
using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

//------------------------------------------------------------------------------
PlannerROSInterfaceBar* PlannerROSInterfaceBar::instance()
{
  static PlannerROSInterfaceBar* instance = new PlannerROSInterfaceBar();
  return instance;
}

//------------------------------------------------------------------------------
PlannerROSInterfaceBar::PlannerROSInterfaceBar(): ToolBar("PlannerROSInterfaceBar"),
                              mes(*MessageView::mainInstance()),
                              os(MessageView::mainInstance()->cout()){

  addSeparator();

  addLabel(("=Planner_ROS_InterfaceBar="));

  addButton(("UpdatePos"), ("Update Position"))->
    sigClicked().connect(bind(&PlannerROSInterfaceBar::onUpdatePosButtonClicked, this));
    
  addButton(("Start_RPS_ServiceServer"), ("Start RPS ServiceServer"))->
    sigClicked().connect(bind(&PlannerROSInterfaceBar::onStartRPSServiceServerButtonClicked, this));

  addButton(("TEST"), ("TEST"))->
    sigClicked().connect(bind(&PlannerROSInterfaceBar::onTestButtonClicked, this));

}

//------------------------------------------------------------------------------
PlannerROSInterfaceBar::~PlannerROSInterfaceBar()
{
}

//------------------------------------------------------------------------------
void set_RPS_MAP(const tms_msg_rp::rps_map_full::ConstPtr& RPS_MAP){
	//~ sub_Map.clear();
	
	//~ x_llimit = RPS_MAP->x_llimit;
	//~ x_ulimit = RPS_MAP->x_ulimit;
	//~ y_llimit = RPS_MAP->y_llimit;
	//~ y_ulimit = RPS_MAP->y_ulimit;
	//~ cell_size = RPS_MAP->cell_size;
	//~ 
	//~ vector<CollisionMapData> temp_map_line;
	//~ CollisionMapData temp_map_d;
	//~ 
	//~ for(unsigned int x=0;x<RPS_MAP->rps_map_x.size();x++){
		//~ temp_map_line.clear();
		//~ for(unsigned int y=0;y<RPS_MAP->rps_map_x[x].rps_map_y.size();y++){
			//~ temp_map_d.object			=	RPS_MAP->rps_map_x[x].rps_map_y[y].object;
			//~ temp_map_d.collision		=	RPS_MAP->rps_map_x[x].rps_map_y[y].object;
			//~ temp_map_d.voronoi			=	RPS_MAP->rps_map_x[x].rps_map_y[y].voronoi;
			//~ temp_map_d.dist_from_obj	=	RPS_MAP->rps_map_x[x].rps_map_y[y].dist_from_obj_f;
			//~ 
			//~ temp_map_line.push_back(temp_map_d);
		//~ }
		//~ sub_Map.push_back(temp_map_line);
	//~ }
	
	PlanBase::instance()->llimitMap[0] = RPS_MAP->x_llimit;
	PlanBase::instance()->ulimitMap[0] = RPS_MAP->x_ulimit;
	PlanBase::instance()->llimitMap[1] = RPS_MAP->y_llimit;
	PlanBase::instance()->ulimitMap[1] = RPS_MAP->y_ulimit;
	PlanBase::instance()->llimitMap[2] = -0.1;
	PlanBase::instance()->ulimitMap[2] = 0.1;
}

//------------------------------------------------------------------------------
void PlannerROSInterfaceBar::onUpdatePosButtonClicked(){
	ros::init(argc, argv, "cnoid_Planner_Get_Info");
	ros::NodeHandle nh;
	ros::ServiceClient commander_to_get_robots_info = nh.serviceClient<tms_msg_db::tmsdb_get_robots_info>("tmsdb_get_robots_info");
	ros::ServiceClient commander_to_get_movable_furnitures_info = nh.serviceClient<tms_msg_db::tmsdb_get_movable_furnitures_info>("tmsdb_get_movable_furnitures_info");
	ros::ServiceClient commander_to_get_person_behavior_info = nh.serviceClient<tms_msg_db::tmsdb_get_person_behavior_info>("tmsdb_get_person_info_3");
	ros::ServiceClient commander_to_get_person_info = nh.serviceClient<tms_msg_db::tmsdb_get_person_info>("tmsdb_get_current_person_info");
	
	PlanBase* tc = PlanBase::instance();
	WagonPlanner* wp = WagonPlanner::instance();
	HumanPlanner* hp = HumanPlanner::instance();
	RPS_Controller* rc = RPS_Controller::instance();
	
	vector<double> pos, rpy, joint_angle;
	
	if(tc->targetArmFinger){
		tms_msg_db::tmsdb_get_robots_info srv_get_r_info;
		if(tc->targetArmFinger->bodyItemRobot->body()->name() == "SmartPal4"){
			srv_get_r_info.request.robots_id = 1;
		}
		if(tc->targetArmFinger->bodyItemRobot->body()->name() == "SmartPal5"){
			srv_get_r_info.request.robots_id = 2;
		}
		if(commander_to_get_robots_info.call(srv_get_r_info)){
			ROS_INFO("Success robots_x = %lf, y = %lf, theta = %lf", srv_get_r_info.response.robots_x,srv_get_r_info.response.robots_y,srv_get_r_info.response.robots_theta);
		}
		else{
			os<<"ROS ERROR : get robot info is failed"<<endl;
			return;
		}
		
		pos.resize(3);
		rpy.resize(3);
		joint_angle.resize(tc->body()->numJoints());
		
		pos[0] = srv_get_r_info.response.robots_x / 1000.0 ;
		pos[1] = srv_get_r_info.response.robots_y / 1000.0 ;
		pos[2] = srv_get_r_info.response.robots_z / 1000.0 ;
		
		rpy[0] = 0.0;
		rpy[1] = 0.0;
		rpy[2] = deg2rad(srv_get_r_info.response.robots_theta);
		
		joint_angle[3] = deg2rad(-10.0);
		joint_angle[11] = deg2rad(10.0);
		
		rc->SetRobotPose(pos, rpy, joint_angle);
	}
	
	if(wp->targetWagon){
		tms_msg_db::tmsdb_get_movable_furnitures_info srv_get_f_info;
		srv_get_f_info.request.furnitures_id = 22;
		if(commander_to_get_movable_furnitures_info.call(srv_get_f_info)){
			ROS_INFO("Success wagon_x = %lf, y = %lf, theta = %lf", srv_get_f_info.response.furniture_x,srv_get_f_info.response.furniture_y,srv_get_f_info.response.furnitures_theta);
			ROS_INFO("Success wagon_width = %lf, depth = %lf, height = %lf", srv_get_f_info.response.furnitures_width,srv_get_f_info.response.furnitures_depth,srv_get_f_info.response.furnitures_height);
		}
		else{
			os<<"ROS ERROR : get wagon info is failed"<<endl;
			return;
		}
	
		wp->targetWagon->size_LongSide_Length = srv_get_f_info.response.furnitures_width;
		wp->targetWagon->size_ShortSide_Length = srv_get_f_info.response.furnitures_depth;
		wp->targetWagon->size_Height = srv_get_f_info.response.furnitures_height;
		
		pos.resize(3);
		rpy.resize(3);
		
		pos[0] = srv_get_f_info.response.furniture_x / 1000.0 ;
		pos[1] = srv_get_f_info.response.furniture_y / 1000.0 ;
		pos[2] = srv_get_f_info.response.furniture_z / 1000.0 ;
		
		rpy[0] = 0.0;
		rpy[1] = 0.0;
		rpy[2] = deg2rad(srv_get_f_info.response.furnitures_theta);
		
		rc->SetWagonPose(pos, rpy);
	}
	
	if(hp->targetHuman){
		tms_msg_db::tmsdb_get_person_behavior_info srv_get_pb_info;
		tms_msg_db::tmsdb_get_person_info srv_get_p_info;
		
		pos.resize(3);
		rpy.resize(3);
		joint_angle.resize(hp->body()->numJoints());
		
		vector<double> def_human_joint_angle, sitting_human_joint_angle;
		def_human_joint_angle.resize(hp->body()->numJoints());
		sitting_human_joint_angle.resize(hp->body()->numJoints());
		
		for(unsigned int i=0;i<hp->body()->numJoints();i++){
			def_human_joint_angle[i] = 0.0;
			sitting_human_joint_angle[i] = 0.0;
		}
		sitting_human_joint_angle[20] = deg2rad(-90.0);
		sitting_human_joint_angle[23] = deg2rad(90.0);
		sitting_human_joint_angle[27] = deg2rad(-90.0);
		sitting_human_joint_angle[30] = deg2rad(90.0);
		
		if(commander_to_get_person_behavior_info.call(srv_get_pb_info)){
			ROS_INFO("%u, %u", srv_get_pb_info.response.id, srv_get_pb_info.response.behavior);
			/*
			person_behavior
			0 消失
			1 歩行
			2 立位静止
			3 椅子付近 静止
			4 椅子着座
			5 ベッド着座
			6 ベッド上 (休息)
			*/
		}
		else{
			os<<"ROS ERROR : get person behavior info is failed"<<endl;
			return;
		}
		
		if((srv_get_pb_info.response.behavior==1)||(srv_get_pb_info.response.behavior==2)||(srv_get_pb_info.response.behavior==3)){
			if(commander_to_get_person_info.call(srv_get_p_info)){
				ROS_INFO("%u, %f, %f, %f", srv_get_p_info.response.id[0], srv_get_p_info.response.x[0], srv_get_p_info.response.y[0], srv_get_p_info.response.theta[0]);
				
				pos[0] = srv_get_p_info.response.x[0] / 1000.0 ;
				pos[1] = srv_get_p_info.response.y[0] / 1000.0 ;
				//~ pos[2] = srv_get_p_info.response.z[0] / 1000.0 ;
				pos[2]  = 847.1 / 1000.0 ;	//human_160
				
				rpy[0] = 0.0;
				rpy[1] = 0.0;
				rpy[2] = deg2rad(srv_get_p_info.response.theta[0]);
				
				joint_angle = def_human_joint_angle;
				
				rc->SetHumanPose(pos, rpy, joint_angle);
			}
			else{
				os<<"ROS ERROR : get person info is failed"<<endl;
				return;
			}
		}
		
		if(srv_get_pb_info.response.behavior==4){
			pos[0] = 1400 / 1000.0 ;
			pos[1] = 2500 / 1000.0 ;
			//~ pos[2] = srv_get_p_info.response.z[0] / 1000.0 ;
			pos[2] = 530 / 1000.0 ;	//human_160
			
			rpy[0] = 0.0;
			rpy[1] = 0.0;
			rpy[2] = deg2rad(-90.0);
			
			joint_angle = sitting_human_joint_angle;
			
			rc->SetHumanPose(pos, rpy, joint_angle);
		}
	}
}

//------------------------------------------------------------------------------
bool start_rps_cnoid_grasp_obj_planning(tms_msg_rp::rps_cnoid_grasp_obj_planning::Request& req, tms_msg_rp::rps_cnoid_grasp_obj_planning::Response& res)
{
	res.success = 0;
	res.robot_joint_angle.clear();
	
	RPS_Controller* rc = RPS_Controller::instance();
	
	ArmFingers* temp_ArmFinger = rc->pb->targetArmFinger;
	rc->pb->targetArmFinger = rc->pb->armsList[2];
	
	vector<double> pos, rpy, joint_angle;
	pos.resize(3);
	rpy.resize(3);
	joint_angle.resize(rc->pb->body()->numJoints());
	
	tms_msg_rp::rps_joint_angle temp_rps_joint_angle;
	temp_rps_joint_angle.joint_angle.clear();
	
	//set robot pose
	pos[0] = req.robot_pos.x / 1000.0 ;
	pos[1] = req.robot_pos.y / 1000.0 ;
	pos[2] = 0.0;
	rpy[0] = 0.0;
	rpy[1] = 0.0;
	rpy[2] = deg2rad(req.robot_pos.th);
	joint_angle.clear();
	for(unsigned int j=0;j<req.pre_robot_joint_angle.joint_angle.size();j++){
		joint_angle.push_back(deg2rad(req.pre_robot_joint_angle.joint_angle[j]));
		temp_rps_joint_angle.joint_angle.push_back(req.pre_robot_joint_angle.joint_angle[j]);
	}
	rc->SetRobotPose(pos, rpy, joint_angle);
	res.robot_joint_angle.push_back(temp_rps_joint_angle);
	
	//set obj pose
	//~ pos[0] = req.obj_pos.x / 1000.0 ;
	//~ pos[1] = req.obj_pos.y / 1000.0 ;
	//~ pos[2] = 0.0;
	//~ rpy[0] = 0.0;
	//~ rpy[1] = 0.0;
	//~ rpy[2] = deg2rad(req.obj_pos.th);
	//~ 
	//~ rc->SetObjectPose(pos, rpy);
	
	vector<vector<double> > out_joint_angle;
	bool result = rc->GraspObjPlan(out_joint_angle);
	
	temp_rps_joint_angle.joint_angle.clear();
	for(unsigned int i=0;i<out_joint_angle.size();i++){
		temp_rps_joint_angle.joint_angle.clear();
		for(unsigned int j=0;j<out_joint_angle[i].size();j++){
			temp_rps_joint_angle.joint_angle.push_back(rad2deg(out_joint_angle[i][j]));
		}
		res.robot_joint_angle.push_back(temp_rps_joint_angle);
	}
	
	rc->pb->targetArmFinger = temp_ArmFinger;
	
	if(result){
		res.success = 1;
		res.message = "Success : rps_cnoid_grasp_obj_plan";
		return true;
	}
	else{
		res.success = 0;
		res.message = "Failed : rps_cnoid_grasp_obj_plan";
		return false;
	}
}

//------------------------------------------------------------------------------
bool start_rps_cnoid_give_obj_planning(tms_msg_rp::rps_cnoid_grasp_obj_planning::Request& req, tms_msg_rp::rps_cnoid_grasp_obj_planning::Response& res)
{
	res.success = 0;
	res.robot_joint_angle.clear();
	
	RPS_Controller* rc = RPS_Controller::instance();
	
	vector<double> pos, rpy, joint_angle, rob_init_joint_angle;
	pos.resize(3);
	rpy.resize(3);
	joint_angle.resize(rc->pb->body()->numJoints());
	rob_init_joint_angle.clear();
	
	Vector3 obj_init_p = rc->pb->object()->p();
	Matrix3 obj_init_R = rc->pb->object()->R();
	Vector3 rob_init_p = rc->pb->body()->link(0)->p();
	Matrix3 rob_init_R = rc->pb->body()->link(0)->R();
	for(unsigned int j=0;j<rc->pb->body()->numJoints();j++){
		rob_init_joint_angle.push_back(rc->pb->body()->joint(j)->q());
	}
	
	tms_msg_rp::rps_joint_angle temp_rps_joint_angle;
	temp_rps_joint_angle.joint_angle.clear();
	
	//set robot pose
	pos[0] = req.robot_pos.x / 1000.0 ;
	pos[1] = req.robot_pos.y / 1000.0 ;
	pos[2] = 0.0;
	rpy[0] = 0.0;
	rpy[1] = 0.0;
	rpy[2] = deg2rad(req.robot_pos.th);
	joint_angle.clear();
	for(unsigned int j=0;j<req.pre_robot_joint_angle.joint_angle.size();j++){
		joint_angle.push_back(deg2rad(req.pre_robot_joint_angle.joint_angle[j]));
		temp_rps_joint_angle.joint_angle.push_back(req.pre_robot_joint_angle.joint_angle[j]);
	}
	rc->SetRobotPose(pos, rpy, joint_angle);
	res.robot_joint_angle.push_back(temp_rps_joint_angle);
	
	//set obj pose
	rc->wp->changeObjectType(rc->pb->targetObject, WagonPlanner::NOT_GRASPING);
	
	pos[0] = req.obj_pos.x / 1000.0 ;
	pos[1] = req.obj_pos.y / 1000.0 ;
	pos[2] = req.obj_pos.z / 1000.0 ;
	rpy[0] = 0.0;
	rpy[1] = 0.0;
	rpy[2] = deg2rad(req.obj_pos.th);
	
	rc->SetObjectPose(pos, rpy);
	
	vector<vector<double> > out_joint_angle;
	bool result = rc->GraspObjPlan(out_joint_angle);
	
	//~ for(unsigned int i=0;i<out_joint_angle.size();i++){
		temp_rps_joint_angle.joint_angle.clear();
		for(unsigned int j=0;j<out_joint_angle[1].size();j++){
			temp_rps_joint_angle.joint_angle.push_back(rad2deg(out_joint_angle[1][j]));
		}
		res.robot_joint_angle.push_back(temp_rps_joint_angle);
	//~ }
	
	//reset pos
	//~ pos[0] = rob_init_p(0);
	//~ pos[1] = rob_init_p(1);
	//~ pos[2] = rob_init_p(2);
	//~ rpy[0] = rpyFromRot(rob_init_R)(0);
	//~ rpy[1] = rpyFromRot(rob_init_R)(1);
	//~ rpy[2] = rpyFromRot(rob_init_R)(2);
	//~ joint_angle.clear();
	//~ for(unsigned int j=0;j<req.pre_robot_joint_angle.joint_angle.size();j++){
		//~ joint_angle.push_back(rob_init_joint_angle[j]);
	//~ }
	//~ rc->SetRobotPose(pos, rpy, joint_angle);
	//~ 
	//~ pos[0] = obj_init_p(0);
	//~ pos[1] = obj_init_p(1);
	//~ pos[2] = obj_init_p(2);
	//~ rpy[0] = rpyFromRot(obj_init_R)(0);
	//~ rpy[1] = rpyFromRot(obj_init_R)(1);
	//~ rpy[2] = rpyFromRot(obj_init_R)(2);
	//~ 
	//~ rc->SetObjectPose(pos, rpy);
	
	if(result){
		res.success = 1;
		res.message = "Success : rps_cnoid_give_obj_plan";
		return true;
	}
	else{
		res.success = 0;
		res.message = "Failed : rps_cnoid_give_obj_plan";
		return false;
	}
}

//------------------------------------------------------------------------------
bool start_rps_cnoid_calc_grasp_wagon_pose(tms_msg_rp::rps_cnoid_grasp_wagon_planning::Request& req, tms_msg_rp::rps_cnoid_grasp_wagon_planning::Response& res)
{
	res.success = 0;
	res.robot_joint_angle.clear();
	
	RPS_Controller* rc = RPS_Controller::instance();
	
	rc->wp->changeWagonType(rc->wp->targetWagon, Wagon::COLLISION);
	
	vector<double> pos, rpy, joint_angle;
	pos.resize(3);
	rpy.resize(3);
	
	//set robot pose
	pos[0] = req.robot_pos.x / 1000.0 ;
	pos[1] = req.robot_pos.y / 1000.0 ;
	pos[2] = 0.0;
	rpy[0] = 0.0;
	rpy[1] = 0.0;
	rpy[2] = deg2rad(req.robot_pos.th);
	joint_angle.clear();
	for(unsigned int j=0;j<req.pre_robot_joint_angle.joint_angle.size();j++){
		joint_angle.push_back(deg2rad(req.pre_robot_joint_angle.joint_angle[j]));
	}
	rc->SetRobotPose(pos, rpy, joint_angle);
	
	//set wagon pose
	pos[0] = req.wagon_pos.x / 1000.0 ;
	pos[1] = req.wagon_pos.y / 1000.0 ;
	pos[2] = 0.0;
	rpy[0] = 0.0;
	rpy[1] = 0.0;
	rpy[2] = deg2rad(req.wagon_pos.th);
	
	rc->SetWagonPose(pos, rpy);
	
	vector<double> out_joint_angle;
	bool result = rc->calcGraspWagonPose(out_joint_angle);
	
	tms_msg_rp::rps_joint_angle temp_rps_joint_angle;
	temp_rps_joint_angle.joint_angle.clear();
	for(unsigned int i=0;i<out_joint_angle.size();i++){
		temp_rps_joint_angle.joint_angle.push_back(rad2deg(out_joint_angle[i]));
	}
	res.robot_joint_angle.push_back(temp_rps_joint_angle);
	
	if(result){
		res.success = 1;
		res.message = "Success : rps_cnoid_calc_grasp_wagon_pose";
		return true;
	}
	else{
		res.success = 0;
		res.message = "Failed : rps_cnoid_calc_grasp_wagon_pose";
		return false;
	}
}

//------------------------------------------------------------------------------
bool start_rps_cnoid_grasp_wagon_plan(tms_msg_rp::rps_cnoid_grasp_wagon_planning::Request& req, tms_msg_rp::rps_cnoid_grasp_wagon_planning::Response& res)
{
	res.success = 0;
	res.robot_joint_angle.clear();
	
	RPS_Controller* rc = RPS_Controller::instance();
	
	vector<double> pos, rpy, joint_angle;
	pos.resize(3);
	rpy.resize(3);
	joint_angle.resize(rc->pb->body()->numJoints());
	
	tms_msg_rp::rps_joint_angle temp_rps_joint_angle;
	
	//set robot pose
	pos[0] = req.robot_pos.x / 1000.0 ;
	pos[1] = req.robot_pos.y / 1000.0 ;
	pos[2] = 0.0;
	rpy[0] = 0.0;
	rpy[1] = 0.0;
	rpy[2] = deg2rad(req.robot_pos.th);
	joint_angle.clear();
	temp_rps_joint_angle.joint_angle.clear();
	for(unsigned int j=0;j<req.pre_robot_joint_angle.joint_angle.size();j++){
		joint_angle.push_back(deg2rad(req.pre_robot_joint_angle.joint_angle[j]));
		temp_rps_joint_angle.joint_angle.push_back(req.pre_robot_joint_angle.joint_angle[j]);
	}
	rc->SetRobotPose(pos, rpy, joint_angle);
	res.robot_joint_angle.push_back(temp_rps_joint_angle);
	
	//set wagon pose
	pos[0] = req.wagon_pos.x / 1000.0 ;
	pos[1] = req.wagon_pos.y / 1000.0 ;
	pos[2] = 0.0;
	rpy[0] = 0.0;
	rpy[1] = 0.0;
	rpy[2] = deg2rad(req.wagon_pos.th);
	
	rc->SetWagonPose(pos, rpy);
	
	int grasp_wagon_type = 0;
	double palm_th = 0.0, rel_th = 0.0;
	vector<vector<double> > out_joint_angle;
	vector<double> rel_control_point_pos;
	bool result = rc->GraspWagonPlan(grasp_wagon_type, out_joint_angle, palm_th, rel_control_point_pos, rel_th);
	
	res.grasp_wagon_type = grasp_wagon_type;
	for(unsigned int i=0;i<out_joint_angle.size();i++){
		temp_rps_joint_angle.joint_angle.clear();
		for(unsigned int j=0;j<out_joint_angle[i].size();j++){
			temp_rps_joint_angle.joint_angle.push_back(rad2deg(out_joint_angle[i][j]));
		}
		res.robot_joint_angle.push_back(temp_rps_joint_angle);
	}
	res.palm_th = rad2deg(palm_th);
	res.rel_control_point_pos.x = rel_control_point_pos[0];
	res.rel_control_point_pos.y = rel_control_point_pos[1];
	res.rel_control_point_pos.th = rad2deg(rel_th);
	
	if(result){
		res.success = 1;
		res.message = "Success : rps_cnoid_grasp_wagon";
		return true;
	}
	else{
		res.success = 0;
		res.message = "Failed : rps_cnoid_grasp_wagon_";
		return false;
	}
}

//------------------------------------------------------------------------------
bool start_rps_cnoid_release_wagon_plan(tms_msg_rp::rps_cnoid_grasp_wagon_planning::Request& req, tms_msg_rp::rps_cnoid_grasp_wagon_planning::Response& res)
{
	res.success = 0;
	res.robot_joint_angle.clear();
	
	RPS_Controller* rc = RPS_Controller::instance();
	
	vector<double> pos, rpy, joint_angle;
	pos.resize(3);
	rpy.resize(3);
	joint_angle.resize(rc->pb->body()->numJoints());
	
	//set robot pose
	pos[0] = req.robot_pos.x / 1000.0 ;
	pos[1] = req.robot_pos.y / 1000.0 ;
	pos[2] = 0.0;
	rpy[0] = 0.0;
	rpy[1] = 0.0;
	rpy[2] = deg2rad(req.robot_pos.th);
	joint_angle.clear();
	for(unsigned int j=0;j<req.pre_robot_joint_angle.joint_angle.size();j++){
		joint_angle.push_back(deg2rad(req.pre_robot_joint_angle.joint_angle[j]));
	}
	rc->SetRobotPose(pos, rpy, joint_angle);
	
	//set wagon pose
	pos[0] = req.wagon_pos.x / 1000.0 ;
	pos[1] = req.wagon_pos.y / 1000.0 ;
	pos[2] = 0.0;
	rpy[0] = 0.0;
	rpy[1] = 0.0;
	rpy[2] = deg2rad(req.wagon_pos.th);
	
	rc->SetWagonPose(pos, rpy);
	
	vector<vector<double> > out_joint_angle;
	bool result = rc->ReleaseWagonPlan(out_joint_angle);
	res.grasp_wagon_type = 0;
	tms_msg_rp::rps_joint_angle temp_rps_joint_angle;
	for(unsigned int i=0;i<out_joint_angle.size();i++){
		temp_rps_joint_angle.joint_angle.clear();
		for(unsigned int j=0;j<out_joint_angle[i].size();j++){
			temp_rps_joint_angle.joint_angle.push_back(rad2deg(out_joint_angle[i][j]));
		}
		res.robot_joint_angle.push_back(temp_rps_joint_angle);
	}
	if(result){
		res.success = 1;
		res.message = "Success : rps_cnoid_release_wagon";
		return true;
	}
	else{
		res.success = 0;
		res.message = "Failed : rps_cnoid_release_wagon_";
		return false;
	}
}

//------------------------------------------------------------------------------
void setPlanDOF(vector<WagonMotionState>& PlanMotionState){
	PlanBase* pb = PlanBase::instance();
	int top = pb->body()->numJoints();
	if(PlanMotionState.size()<1){
		cout<<"Please Add Plan Motion"<<endl;
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
			//~ PlanMotionState[i].pathPlanDOF.push_back(top+5);
		}
		if( fabs(PlanMotionState[i].rpy[2] - PlanMotionState[i+1].rpy[2]) > 1.e-10){
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

//------------------------------------------------------------------------------
bool start_rps_cnoid_PRM_plan(tms_msg_rp::rps_cnoid_PRM_planning::Request& req, tms_msg_rp::rps_cnoid_PRM_planning::Response& res)
{
	res.success = 0;
	
	RPS_Controller* rc = RPS_Controller::instance();
	rc->wp->wagonMotionSeq.clear();
	
	vector<double> pos(3), rpy(3), joint_angle;
	joint_angle.clear();
	
	WagonMotionState initMotionState = rc->wp->getMotionState(), tempMotionState;
	
	for(unsigned int i=0;i<req.in_motion_state.size();i++){
		rc->wp->changeWagonType(rc->wp->targetWagon, Wagon::WagonType(req.in_motion_state[i].pushing_wagon_state));
		rc->wp->changeObjectType(rc->pb->targetObject, WagonPlanner::ObjectType(req.in_motion_state[i].grasping_object_state));
		
		pos[0] = req.in_motion_state[i].robot_pos.x / 1000.0;
		pos[1] = req.in_motion_state[i].robot_pos.y / 1000.0;
		pos[2] = req.in_motion_state[i].robot_pos.z / 1000.0;
		rpy[0] = deg2rad(req.in_motion_state[i].robot_pos.roll);
		rpy[1] = deg2rad(req.in_motion_state[i].robot_pos.pitch);
		rpy[2] = deg2rad(req.in_motion_state[i].robot_pos.yaw);
		joint_angle.clear();
		for(unsigned int j=0;j<req.in_motion_state[i].robot_joint_angle.joint_angle.size();j++){
			joint_angle.push_back(deg2rad(req.in_motion_state[i].robot_joint_angle.joint_angle[j]));
		}
		rc->SetRobotPose(pos, rpy, joint_angle);
		
		pos[0] = req.in_motion_state[i].wagon_pos.x / 1000.0;
		pos[1] = req.in_motion_state[i].wagon_pos.y / 1000.0;
		pos[2] = req.in_motion_state[i].wagon_pos.z / 1000.0;
		rpy[0] = deg2rad(req.in_motion_state[i].wagon_pos.roll);
		rpy[1] = deg2rad(req.in_motion_state[i].wagon_pos.pitch);
		rpy[2] = deg2rad(req.in_motion_state[i].wagon_pos.yaw);
		rc->SetWagonPose(pos, rpy);

		pos[0] = req.in_motion_state[i].object_pos.x / 1000.0;
		pos[1] = req.in_motion_state[i].object_pos.y / 1000.0;
		pos[2] = req.in_motion_state[i].object_pos.z / 1000.0;
		rpy[0] = deg2rad(req.in_motion_state[i].object_pos.roll);
		rpy[1] = deg2rad(req.in_motion_state[i].object_pos.pitch);
		rpy[2] = deg2rad(req.in_motion_state[i].object_pos.yaw);
		rc->SetObjectPose(pos, rpy);
		
		tempMotionState = rc->wp->getMotionState();
		//~ tempMotionState.graspingState = PlanBase::GraspingStates(req.in_motion_state[i].grasping_object_state);
		//~ tempMotionState.object_type = WagonMotionState::ObjectType(req.in_motion_state[i].grasping_object_state);
		
		rc->wp->wagonMotionSeq.push_back(tempMotionState);
	}
	
	setPlanDOF(rc->wp->wagonMotionSeq);
	
	rc->wp->setMotionState(initMotionState);
		
	WagonTrajectoryPlanner* WagontrajectoryPlanner_ = new WagonTrajectoryPlanner();
	
	bool result = WagontrajectoryPlanner_->doWagonTrajectoryPlanning();
	//~ cout<<WagontrajectoryPlanner_->wagon_motionSeq.size()<<endl;

	res.out_motion_state.clear();
	tms_msg_rp::rps_cnoid_wagon_motion_state temp_state;
	temp_state.robot_id = req.in_motion_state[0].robot_id;
	temp_state.wagon_id = req.in_motion_state[0].wagon_id;
	temp_state.object_id = req.in_motion_state[0].object_id;
	if(result){
		res.success = 1;
		res.message = "Success : rps_cnoid_PRM";
		for(unsigned int i=0;i<WagontrajectoryPlanner_->wagon_motionSeq.size();i++){
			temp_state.robot_pos.x = WagontrajectoryPlanner_->wagon_motionSeq[i].pos[0] * 1000.0;
			temp_state.robot_pos.y = WagontrajectoryPlanner_->wagon_motionSeq[i].pos[1] * 1000.0;
			temp_state.robot_pos.z = WagontrajectoryPlanner_->wagon_motionSeq[i].pos[2] * 1000.0;
			temp_state.robot_pos.roll = rad2deg(WagontrajectoryPlanner_->wagon_motionSeq[i].rpy[0]);
			temp_state.robot_pos.pitch = rad2deg(WagontrajectoryPlanner_->wagon_motionSeq[i].rpy[1]);
			temp_state.robot_pos.yaw = rad2deg(WagontrajectoryPlanner_->wagon_motionSeq[i].rpy[2]);
			temp_state.robot_pos.th = rad2deg(WagontrajectoryPlanner_->wagon_motionSeq[i].rpy[2]);
			
			temp_state.robot_joint_angle.joint_angle.clear();
			for(unsigned int j=0;j<WagontrajectoryPlanner_->wagon_motionSeq[i].jointSeq.size();j++){
				temp_state.robot_joint_angle.joint_angle.push_back(rad2deg(WagontrajectoryPlanner_->wagon_motionSeq[i].jointSeq[j]));
			}
			
			//~ temp_state.grasping_object_state = WagontrajectoryPlanner_->wagon_motionSeq[i].ObjectType;
			
			res.out_motion_state.push_back(temp_state);
		}
		
		return true;
	}
	else{
		res.success = 0;
		res.message = "Failed : rps_cnoid_PRM";
		return false;
	}
}

//------------------------------------------------------------------------------
void PlannerROSInterfaceBar::onStartRPSServiceServerButtonClicked(){
	os<<"Start cnoid_RPS_ServiceServer"<<endl;
	
	RPS_Controller* rc = RPS_Controller::instance();
	rc->wp->initialCollision();
	rc->hp->initialCollision();
	
	ros::init(argc, argv, "cnoid_RPS_ServiceServer");
	ros::NodeHandle nh;
	
	ros::Subscriber	rps_map_subscriber = nh.subscribe("rps_map_data", 1, set_RPS_MAP);
	
	ros::ServiceServer service_rps_cnoid_grasp_obj_planning = nh.advertiseService("rps_cnoid_grasp_obj_planning", start_rps_cnoid_grasp_obj_planning);
	ros::ServiceServer service_rps_cnoid_give_obj_planning = nh.advertiseService("rps_cnoid_give_obj_planning", start_rps_cnoid_give_obj_planning);
	ros::ServiceServer service_rps_cnoid_calc_grasp_wagon_pose = nh.advertiseService("rps_cnoid_calc_grasp_wagon_pose", start_rps_cnoid_calc_grasp_wagon_pose);
	ros::ServiceServer service_rps_cnoid_grasp_wagon_planning = nh.advertiseService("rps_cnoid_grasp_wagon_planning", start_rps_cnoid_grasp_wagon_plan);
	ros::ServiceServer service_rps_cnoid_release_wagon_planning = nh.advertiseService("rps_cnoid_release_wagon_planning", start_rps_cnoid_release_wagon_plan);
	ros::ServiceServer service_rps_cnoid_PRM_planning = nh.advertiseService("rps_cnoid_PRM_planning", start_rps_cnoid_PRM_plan);
	
	ros::spin();
	os<<"End cnoid_RPS_ServiceServer"<<endl;
}

//------------------------------------------------------------------------------
void PlannerROSInterfaceBar::onTestButtonClicked(){
	//~ ros::init(argc, argv, "cnoid_Planner_ROS_Interface");
	//~ ros::NodeHandle nh;
	//~ ros::ServiceServer service_set = nh.advertiseService("skeleton_pose_set", pose_set);
	//~ 
	//~ ros::spin();
	
	cout<<"test"<<endl;
}
