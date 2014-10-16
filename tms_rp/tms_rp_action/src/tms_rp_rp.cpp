#include <tms_rp_rp.h>

//------------------------------------------------------------------------------
using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

double sp5arm_init_arg[26] = {	0.0, 0.0, 1.0, 1.0,	/*waist*/
									0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 10.0, 10.0,/*right arm*/
									0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 10.0, 10.0 /*left arm*/};	//degree

//------------------------------------------------------------------------------
tms_rp::TmsRpSubtask* tms_rp::TmsRpSubtask::instance()
{
  static tms_rp::TmsRpSubtask* instance = new tms_rp::TmsRpSubtask();
  return instance;
}

tms_rp::TmsRpSubtask::TmsRpSubtask(): ToolBar("TmsRpSubtask"),
		os(MessageView::mainInstance()->cout()), tac(*TmsRpController::instance()) {
	sid = 100000;

	static ros::NodeHandle nh1;
	rp_subtask_server = nh1.advertiseService("rp_cmd", &TmsRpSubtask::subtask, this);
	get_data_client            = nh1.serviceClient<tms_msg_db::TmsdbGetData>("/tms_db_reader/dbreader");
	sp5_control_client         = nh1.serviceClient<tms_msg_rc::rc_robot_control>("sp5_control");
	sp5_virtual_control_client = nh1.serviceClient<tms_msg_rc::rc_robot_control>("sp5_virtual_control");
	kxp_virtual_control_client = nh1.serviceClient<tms_msg_rc::rc_robot_control>("kxp_virtual_control");
	kxp_mbase_client           = nh1.serviceClient<tms_msg_rc::tms_rc_pmove>("pmove");
	kobuki_virtual_control_client = nh1.serviceClient<tms_msg_rc::rc_robot_control>("kobuki_virtual_control");
	voronoi_path_planning_client  = nh1.serviceClient<tms_msg_rp::rps_voronoi_path_planning>("rps_voronoi_path_planning");
	give_obj_client   = nh1.serviceClient<tms_msg_rp::rps_goal_planning>("rps_give_obj_pos_planning");
	refrigerator_client = nh1.serviceClient<tms_msg_rs::rs_home_appliances>("refrigerator_controller");
	}

tms_rp::TmsRpSubtask::~TmsRpSubtask()
{
}

bool tms_rp::TmsRpSubtask::get_robot_pos(bool type, int robot_id, std::string& robot_name, tms_msg_rp::rps_voronoi_path_planning& rp_srv) {
	tms_msg_db::TmsdbGetData srv;

	// get robot's present location from TMSDB
	srv.request.tmsdb.id = robot_id;
	if (type == true) { // production version
		srv.request.tmsdb.sensor = 3001; // vicon system
	} else { // simulation
		srv.request.tmsdb.sensor = 3005; // fake odometry
	}

	if(get_data_client.call(srv)) {
		robot_name = srv.response.tmsdb[0].name;
		rp_srv.request.start_pos.x = srv.response.tmsdb[0].x;
		rp_srv.request.start_pos.y = srv.response.tmsdb[0].y;
		rp_srv.request.start_pos.z = 0.0;
		rp_srv.request.start_pos.th = srv.response.tmsdb[0].ry; // =yaw
		rp_srv.request.start_pos.roll = 0.0; // srv.response.tmsdb[0].rr
		rp_srv.request.start_pos.pitch = 0.0; // srv.response.tmsdb[0].rp
		rp_srv.request.start_pos.yaw = srv.response.tmsdb[0].ry; // srv.response.tmsdb[0].ry
		// set odom for production version
		if (type == true && robot_id == 2002) {
		double arg[1] = {0.0};
		sp5_control(true, UNIT_ALL, CMD_GETSTATE, 1, arg);
		}
	} else {
		ROS_INFO("Failed to call service tms_db_get_current_robot_data.\n");
		// use odometry when the robot cannot get data from vicon system
		if (type == true) {
			srv.request.tmsdb.sensor = 3003; // odometory
			if (get_data_client.call(srv)) {
				robot_name = srv.response.tmsdb[0].name;
				rp_srv.request.start_pos.x = srv.response.tmsdb[0].x;
				rp_srv.request.start_pos.y = srv.response.tmsdb[0].y;
				rp_srv.request.start_pos.z = 0.0;
				rp_srv.request.start_pos.th = srv.response.tmsdb[0].ry;
				rp_srv.request.start_pos.roll = 0.0; // srv.response.tmsdb[0].rr
				rp_srv.request.start_pos.pitch = 0.0; // srv.response.tmsdb[0].rp
				rp_srv.request.start_pos.yaw = 0.0; // srv.response.tmsdb[0].ry
			} else {
				ROS_INFO("Failed to get robot current data\n");
				return false;
			}
		}
		return false;
	}
}

bool tms_rp::TmsRpSubtask::subtask(tms_msg_rp::rp_cmd::Request &req,
                      tms_msg_rp::rp_cmd::Response &res)
{
	bool type = req.type;
	int command = req.command;
	int robot_id = req.robot_id;
	int arg_type = (int)req.arg.at(0);
	grasp::TmsRpBar::grasping = 0;

	tms_msg_db::TmsdbGetData srv;
	tms_msg_rp::rps_voronoi_path_planning rp_srv;

	switch (command) {
		case 9001:
		{// move
			ROS_INFO("[tms_rp]move command\n");

			rp_srv.request.robot_id = robot_id;
			// init arm
			if (robot_id == 2002 && type == true) {
				sp5_control(type, UNIT_ARM_R, CMD_MOVE_ABS , 8, sp5arm_init_arg+4);
				sp5_control(type, UNIT_LUMBA, CMD_MOVE_REL, 4, sp5arm_init_arg);
				sp5_control(type, UNIT_GRIPPER_R, CMD_MOVE_ABS, 3, sp5arm_init_arg+12);
			}
			std::string robot_name("");
			get_robot_pos(type, robot_id, robot_name, rp_srv);
			ROS_INFO("[tms_rp]move command\n");

			srv.request.tmsdb.id = arg_type;
			if (arg_type == -1) { // move (x,y,th)
				ROS_INFO("[tms_rp]move command\n");
				rp_srv.request.goal_pos.x = req.arg.at(1);
				rp_srv.request.goal_pos.y = req.arg.at(2);
				rp_srv.request.goal_pos.z = 0.0;
				rp_srv.request.goal_pos.th = req.arg.at(3);
				rp_srv.request.goal_pos.roll = 0.0;
				rp_srv.request.goal_pos.pitch = 0.0;
				rp_srv.request.goal_pos.yaw = req.arg.at(3);
			} else if (arg_type > 1000 && arg_type < 2000) {
				ROS_INFO("Argument's IDtype is Person.\n");
			} else if (arg_type > 2000 && arg_type < 3000) { // RobotID
				ROS_INFO("Argument's IDtype is Robot. Please retry.\n");
				return false;
			} else if (arg_type > 3000 && arg_type < 4000) { // SensorID
				ROS_INFO("Argument's IDtype is Sensor. Please retry.\n");
				return false;
			} else if (arg_type > 6000 && arg_type < 7000) { // FurnitureID
				ROS_INFO("Argument's IDtype is Furniture.\n");
				srv.request.tmsdb.id = arg_type + sid;
				if(get_data_client.call(srv)) {
					// analyze etcdata and get goal position
					std::string etcdata = srv.response.tmsdb[0].etcdata;
					std::vector<std::string> v_etcdata;
					v_etcdata.clear();
					boost::split(v_etcdata, etcdata, boost::is_any_of(";"));

					int i = 0;
					while (robot_name != v_etcdata.at(i)) {
						i = i + 2;
						if (i >= v_etcdata.size()) {
							ROS_ERROR("This robot cannot move to the slate point!\n");
							return false;
						}
					}
					std::string argdata = v_etcdata.at(i+1);
					std::vector<std::string> v_argdata;
					v_argdata.clear();
					boost::split(v_argdata, argdata, boost::is_any_of(",")); // v_argdata[0, 1, 2] = [goal_x, goal_y, goal_th]

					if (v_argdata.size() == 3) {
						// string to double
						std::stringstream ss;
						double d_x, d_y, d_th;

						// goal_x
						ss << v_argdata.at(0);
						ss >> d_x;
						rp_srv.request.goal_pos.x = d_x;
						ROS_INFO("goal_x = [%f]  ", d_x);

						// goal_y
						ss.clear();
						ss.str("");
						ss << v_argdata.at(1);
						ss >> d_y;
						rp_srv.request.goal_pos.y = d_y;
						ROS_INFO("goal_y = [%f]  ", d_y);

						// goal_th
						ss.clear();
						ss.str("");
						ss << v_argdata.at(2);
						ss >> d_th;
						rp_srv.request.goal_pos.th = d_th;
						ROS_INFO("goal_th = [%f]\n", d_th);

						rp_srv.request.goal_pos.z = 0.0;
						rp_srv.request.goal_pos.roll = 0.0;
						rp_srv.request.goal_pos.pitch = 0.0;
						rp_srv.request.goal_pos.yaw = 0.0;
					} else {
						ROS_ERROR("There are incorrect data in DB! Check furniture's etcdata!!\n");
						return false;
					}

				} else {
		    		ROS_INFO("Failed to call service tms_db_get_current_furniture_data.\n");
		    		res.result = 0; // false
		    		return false;
		    	}
			} else if (arg_type > 7000 && arg_type < 8000) { // ObjectID
				ROS_INFO("Argument IDtype is Object%d!\n", arg_type);
				if(get_data_client.call(srv)) {
					int index=0;
					for (int c=0; c<srv.response.tmsdb.size()-1; c++) {
						if (srv.response.tmsdb[c].time < srv.response.tmsdb[c+1].time)
							index =c+1;
					}
					if (srv.response.tmsdb[index].place > 6000 && srv.response.tmsdb[index].place < 7000) {
						srv.request.tmsdb.id = srv.response.tmsdb[index].place + sid;
						if(get_data_client.call(srv)) {
							std::string etcdata = srv.response.tmsdb[0].etcdata;
							ROS_INFO("etc_data = %s", etcdata.c_str());
							std::vector<std::string> v_etcdata;
							v_etcdata.clear();
							boost::split(v_etcdata, etcdata, boost::is_any_of(";"));
							for (int i=0; i<v_etcdata.size(); i++) {
								ROS_INFO("v_etcdata[%d]=%s", i, v_etcdata.at(i).c_str());
							}

							int i = 0;
							while (robot_name != v_etcdata.at(i)) {
								i = i + 2;
								if (i >= v_etcdata.size()) {
									ROS_ERROR("This robot cannot move to the slate point!\n");
									return false;
								}
							}
							std::string argdata = v_etcdata.at(i+1);
							std::vector<std::string> v_argdata;
							v_argdata.clear();
							boost::split(v_argdata, argdata, boost::is_any_of(",")); // v_argdata[0, 1, 2] = [goal_x, goal_y, goal_th]

							if (v_argdata.size() == 3) {
								// string to double
								std::stringstream ss;
								double d_x, d_y, d_th;

								// goal_x
								ss << v_argdata.at(0);
								ss >> d_x; // string to double
								rp_srv.request.goal_pos.x = d_x;
								ROS_INFO("goal_x = [%f]  ", d_x);

								// goal_y
								ss.clear();
								ss.str("");
								ss << v_argdata.at(1);
								ss >> d_y;
								rp_srv.request.goal_pos.y = d_y;
								ROS_INFO("goal_y = [%f]  ", d_y);

								// goal_th
								ss.clear();
								ss.str("");
								ss << v_argdata.at(2);
								ss >> d_th;
								rp_srv.request.goal_pos.th = d_th;
								ROS_INFO("goal_th = [%f]  ", d_th);

								rp_srv.request.goal_pos.z = 0.0;
								rp_srv.request.goal_pos.roll = 0.0;
								rp_srv.request.goal_pos.pitch = 0.0;
								rp_srv.request.goal_pos.yaw = 0.0;
							} else {
								ROS_ERROR("There are incorrect data in DB! Check furniture's etcdata!!\n");
								return false;
							}

						} else {
				    		ROS_INFO("Failed to call service tms_db_get_current_furniture_data.\n");
				    		res.result = 0; // false
				    		return true;
				    	}
					} else {
						ROS_ERROR("Object is in unexpected place!\n");
						return false;
					}
				}
			} else {
				ROS_INFO("Cannot get goal candidate!\n");
				return false;
			}

			ROS_INFO("[tms_rp]move command\n");
			// =====call service "voronoi_path_planning"=====
			if (voronoi_path_planning_client.call(rp_srv)) {
				os << "result: " << rp_srv.response.success << " message: " << rp_srv.response.message << endl;

				  // call virtual_controller
				  if (rp_srv.response.VoronoiPath.size()!=0) {
				    	switch (robot_id) {
				    	case 2002: // smartpal5_1
				    	{
				    		double arg[3];
				    		for (int i=0; i< rp_srv.response.VoronoiPath.size(); i++) {
					    		arg[0] = rp_srv.response.VoronoiPath[i].x;
					    		arg[1] = rp_srv.response.VoronoiPath[i].y;
					    		arg[2] = rp_srv.response.VoronoiPath[i].th;
					    		callSynchronously(bind(&TmsRpSubtask::sp5_control,this, type, UNIT_VEHICLE, CMD_MOVE_ABS, 3, arg));
					    		if (type == true) sp5_control(type, UNIT_ALL, CMD_GETSTATE, 1, arg); // set_odom
					    		else {
					    			callSynchronously(bind(&grasp::TmsRpBar::onSimulationInfoButtonClicked,grasp::TmsRpBar::instance()));
					    			sleep(1); //temp
					    		}
				    		}
				    		break;
				    	}
				    	case 2005: //kobuki
				    	{
				    		for (int i=0; i< rp_srv.response.VoronoiPath.size(); i++) {
				    			tms_msg_rc::rc_robot_control kobuki_srv;
				    			kobuki_srv.request.unit = 1;
				    			kobuki_srv.request.cmd = 15;
				    			kobuki_srv.request.arg.resize(3);
				    			kobuki_srv.request.arg[0] = rp_srv.response.VoronoiPath[i].x;
				    			kobuki_srv.request.arg[1] = rp_srv.response.VoronoiPath[i].y;
				    			kobuki_srv.request.arg[2] = rp_srv.response.VoronoiPath[i].th;
					    		if (kobuki_virtual_control_client.call(kobuki_srv)) ROS_INFO("result: %d", kobuki_srv.response.result);
					    		else                  ROS_ERROR("Failed to call service kobuki_move");

					    		if (type == true) {} // set odom
					    		else {
					    			callSynchronously(bind(&grasp::TmsRpBar::onSimulationInfoButtonClicked,grasp::TmsRpBar::instance()));
					    			sleep(1); //temp
					    		}
				    		}
				    		break;
				    	}
				    	case 2006: // kxp
				    	{
				    		for (int i=0; i< rp_srv.response.VoronoiPath.size(); i++) {
				    			tms_msg_rc::tms_rc_pmove kxp_srv;
				    			kxp_srv.request.w_x = rp_srv.response.VoronoiPath[i].x;
				    			kxp_srv.request.w_y = rp_srv.response.VoronoiPath[i].y;
				    			kxp_srv.request.w_th = rp_srv.response.VoronoiPath[i].th;
					    		if (kxp_mbase_client.call(kxp_srv)) ROS_INFO("result: %d", kxp_srv.response.success);
					    		else                  ROS_ERROR("Failed to call service kxp_mbase");

					    		if (type == true) {} // set odom
					    		else {
					    			callSynchronously(bind(&grasp::TmsRpBar::onSimulationInfoButtonClicked,grasp::TmsRpBar::instance()));
					    			sleep(1); //temp
					    		}
				    		}
				    		break;
				    	}
				    	default:
				    	{
				    		ROS_ERROR("No such robot in TMS_DB!\n");
				    		return false;
				    	}
				    	}
				  } else {
					os << "Noting rps_path_planning data" << endl;
					return false;
				}
			} else {
				ROS_ERROR("Failed to call service rps_path_planning\n");
				return false;
			}
			res.result=1;
			return true;
			break;
		}
		case 9002:
		{   // grasp
			ROS_INFO("[tms_rp]grasp command\n");

			grasp::TmsRpBar::planning_mode = 1;
			if (robot_id == 2002 && type == true) {
				sp5_control(type, UNIT_ARM_R, CMD_MOVE_ABS , 8, sp5arm_init_arg+4);
				sp5_control(type, UNIT_LUMBA, CMD_MOVE_REL, 4, sp5arm_init_arg);
				sp5_control(type, UNIT_GRIPPER_R, CMD_MOVE_ABS, 3, sp5arm_init_arg+12);
			}

			grasp::PlanBase *pb = grasp::PlanBase::instance();
			std::vector<pathInfo> trajectory;
			int state;
			std::vector<double> begin;
			std::vector<double> end;
			// object's position
			std::vector<double> obj_pos;
			std::vector<double> obj_rot;
//			cnoid::Vector3 obj_pos;
//			cnoid::Matrix3 obj_rot;

			grasp::TmsRpController trc;

			// SET OBJECT objectID : 7001~7999
			if(arg_type > 7000 && arg_type < 8000) {
				srv.request.tmsdb.id = arg_type;
				if (get_data_client.call(srv)) {
					int ref_i=0;
					for (int j=0; j<srv.response.tmsdb.size()-1; j++) {
						if (srv.response.tmsdb[j].time < srv.response.tmsdb[j+1].time) ref_i = j+1;
					}
					std::string obj_name = srv.response.tmsdb[ref_i].name;
					cnoid::BodyItemPtr item = trc.objTag2Item()[obj_name];
					grasp::PlanBase::instance()->SetGraspedObject(item);
					ROS_INFO("%s is grasping object.\n", grasp::PlanBase::instance()->targetObject->bodyItemObject->name().c_str());
					// set environment
//						int furniture_id = srv.response.tmsdb[ref_i].place;
//						cout << furniture_id << endl;
//						if (furniture_id > 6000 && furniture_id < 7000) {
//							srv.request.tmsdb.id = furniture_id;
//							if(get_data_client.call(srv)) {
//								std::string furniture_name = srv.response.tmsdb[0].name;
//								cnoid::BodyItemPtr item2 = trc.objTag2Item()[furniture_name];
//								grasp::PlanBase::instance()->SetEnvironment(item2);
//								ROS_INFO("%s is sorrounding environment.\n", furniture_name.c_str());
//							}
//						}
				} else {
					ROS_ERROR("Cannot get object's position\n");
					return false;
				}
			} else {
				ROS_INFO("Robot cannot grasp this object! Wrong id:%d\n", arg_type);
			}

			if( !pb->targetObject || !pb->robTag2Arm.size()) {
				os <<  "set object and robot" << endl;
				return false;
			}
			for(int i=0;i<pb->bodyItemRobot()->body()->numJoints();i++){ // If initial position is not collided, it is stored as
				double q = pb->bodyItemRobot()->body()->joint(i)->q(); // present position
				begin.push_back(q);
				end.push_back(q);
			}
			cout << "set begin and end posture." << endl;

			grasp::TmsRpController::instance()->setTolerance(0.05);
			cout << "set tolerance." << endl;
			grasp::TmsRpController::instance()->graspPathPlanStart_(0, begin, end, pb->targetArmFinger->name,
					pb->targetObject->name(), 1, &trajectory, &state, &obj_pos, &obj_rot);

			double arg[13];
			cnoid::Vector3 rotation, r_rotation;

			arg[0] = trajectory.back().robotPos(0)*1000; // m -> mm
			arg[1] = trajectory.back().robotPos(1)*1000;
			r_rotation = grasp::rpyFromRot(trajectory.back().robotOri);
			arg[2] = rad2deg(r_rotation(2));
			arg[3] = 2;

			pb->calcForwardKinematics();
			pb->targetObject->objVisPos = pb->object()->p();
			pb->targetObject->objVisRot = pb->object()->R();

			rotation = grasp::rpyFromRot(pb->targetObject->objVisRot);
			ROS_INFO("object_x=%fm,y=%fm, z=%f\n", pb->targetObject->objVisPos(0), pb->targetObject->objVisPos(1), pb->targetObject->objVisPos(2));
			ROS_INFO("object_rr=%f,rp=%f, ry=%f\n", rotation(0), rotation(1), rotation(2));

			arg[4] = 7001; // DBに，object_nameに一致するIDを返す命令を追加してもらう.
			arg[5] = pb->targetObject->objVisPos(0)*1000; // m -> mm;
			arg[6] = pb->targetObject->objVisPos(1)*1000;
			arg[7] = pb->targetObject->objVisPos(2)*1000;
			arg[8] = rad2deg(rotation(0));
			arg[9] = rad2deg(rotation(1));
			arg[10] = rad2deg(rotation(2));

			arg[11] = robot_id;// place
			arg[12] = 1;// state

			switch (robot_id) {
			case 2002:
			{
				callSynchronously(bind(&TmsRpSubtask::sp5_control,this, type, UNIT_ALL, CMD_SYNC_OBJ, 13, arg));

				if (type == true) sp5_control(type, UNIT_ALL, CMD_GETSTATE, 1, arg); // set_odom
				else {
					callSynchronously(bind(&grasp::TmsRpBar::onSimulationInfoButtonClicked,grasp::TmsRpBar::instance()));
					sleep(1);
				}
				break;
			}
			case 2006:
			{
				callSynchronously(bind(&TmsRpSubtask::kxp_control,this, type, UNIT_ALL, CMD_SYNC_OBJ, 13, arg));

				if (type == false) {
					callSynchronously(bind(&grasp::TmsRpBar::onSimulationInfoButtonClicked,grasp::TmsRpBar::instance()));
					sleep(1);
				}
				break;
			}
			default:
				break;
			}
			grasp::TmsRpBar::planning_mode = 0;

			break;
		}
		case 9003:
		{   // give
			ROS_INFO("[tms_rp]give command\n");
			grasp::TmsRpBar::planning_mode = 1;
			grasp::TmsRpBar::grasping = 1; // switching model

			// call service for give_obj_planning
			tms_msg_rp::rps_goal_planning gop_srv;
			gop_srv.request.robot_id = robot_id;
			gop_srv.request.target_id = arg_type; // person's ID
			ROS_INFO("robot_id:%d, person_id:%d\n",gop_srv.request.robot_id, gop_srv.request.target_id);

			double goal_arg[19] = {-1,-1,-1,-1,-1,-1,0,0,90,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
			double goal_joint[18];

			if (give_obj_client.call(gop_srv)) {
				if (gop_srv.response.goal_pos.size() != 0) {
					goal_arg[0] = gop_srv.response.goal_pos[0].x;
					goal_arg[1] = gop_srv.response.goal_pos[0].y;
					goal_arg[2] = gop_srv.response.goal_pos[0].th;
					ROS_INFO("goal_x:%fmm,goal_y:%fmm,goal_th:%fdeg\n", goal_arg[0], goal_arg[1], goal_arg[2]);
					goal_arg[3] = gop_srv.response.target_pos.x;
					goal_arg[4] = gop_srv.response.target_pos.y;
					goal_arg[5] = gop_srv.response.target_pos.z;
				}
				// robot's joint angles(j0~j17)
				for (int j=0; j<gop_srv.response.joint_angle_array.at(0).joint_angle.size(); j++)
					goal_joint[j] = gop_srv.response.joint_angle_array.at(0).joint_angle.at(j);
				for (int j=0; j<9; j++) goal_arg[j+9] = goal_joint[j];
			} else {
				ROS_INFO("Cannot call give_object_service!\n");
				return false;
			}

			// move base
			rp_srv.request.robot_id = robot_id;
			std::string robot_name("");
			get_robot_pos(type, robot_id, robot_name, rp_srv);

			rp_srv.request.goal_pos.x = goal_arg[0];
			rp_srv.request.goal_pos.y = goal_arg[1];
			rp_srv.request.goal_pos.th = goal_arg[2];

			switch (robot_id) {
			case 2002: // smartpal5_1
			{
				if (voronoi_path_planning_client.call(rp_srv)) {
					os << "result: " << rp_srv.response.success << " message: " << rp_srv.response.message << endl;
					if (rp_srv.response.VoronoiPath.size()!=0) {
						double arg[13];
						grasp::PlanBase *pb = grasp::PlanBase::instance();

						pb->setGraspingState(grasp::PlanBase::GRASPING);
						for (int i=0; i< rp_srv.response.VoronoiPath.size(); i++) {
							arg[0] = rp_srv.response.VoronoiPath[i].x;
							arg[1] = rp_srv.response.VoronoiPath[i].y;
							arg[2] = rp_srv.response.VoronoiPath[i].th;

							callSynchronously(bind(&TmsRpSubtask::sp5_control,this, type, UNIT_VEHICLE, CMD_MOVE_ABS, 3, arg));
				    		if (type == true) sp5_control(type, UNIT_ALL, CMD_GETSTATE, 1, arg); // set_odom
				    		else {
				    			double rPosX = arg[0]/1000;
				    			double rPosY = arg[1]/1000;
				    			double rPosZ = 0;
				    			cnoid::Vector3 rpy (0, 0, deg2rad(arg[2]));
				    			cnoid::Matrix3 rot = grasp::rotFromRpy(rpy);

				    			callSynchronously(bind(&grasp::TmsRpController::disappear,tac,"smartpal5_1"));
				    			callSynchronously(bind(&grasp::TmsRpController::setPos,tac,"smartpal5_1",cnoid::Vector3(rPosX,rPosY,rPosZ), rot));
				    		}

				    		pb->bodyItemRobot()->body()->calcForwardKinematics();

				    		cnoid::Vector3 position;
							cnoid::Vector3 rotation;
				    		rotation = grasp::rpyFromRot(pb->fingers(0)->tip->R()*(pb->targetArmFinger->objectPalmRot));
				    		position = pb->fingers(0)->tip->p()+pb->fingers(0)->tip->R()*pb->targetArmFinger->objectPalmPos;
				    		//callSynchronously(bind(&grasp::PlanBase::flush,pb));

							ROS_INFO("robot_x=%f,y=%f,z=%f", arg[0], arg[1], arg[2]);
							ROS_INFO("object_x=%f,y=%f,z=%f,rr=%f,rp=%f,ry=%f\n", position(0), position(1), position(2), rad2deg(rotation(0)), rad2deg(rotation(1)), rad2deg(rotation(2)));

							tms_msg_db::TmsdbGetData name_srv;
							name_srv.request.tmsdb.name = pb->targetObject->bodyItemObject->name();
							if(get_data_client.call(name_srv))
							arg[3] = 2; // robot_state
							arg[4] = (double)name_srv.response.tmsdb[0].id; // obj_id
							arg[5] = position(0)*1000; // m -> mm;
							arg[6] = position(1)*1000;
							arg[7] = position(2)*1000;
							arg[8] = rad2deg(rotation(0));
							arg[9] = rad2deg(rotation(1));
							arg[10] = rad2deg(rotation(2));
							arg[11] = robot_id;
							arg[12] = 2; // obj_state
							callSynchronously(bind(&TmsRpSubtask::sp5_control,this, type, UNIT_ALL, CMD_SYNC_OBJ, 13, arg));

				    		if (type == true) sp5_control(type, UNIT_ALL, CMD_GETSTATE, 1, arg); // set_odom
				    		else {
				    			sleep(1);
				    			callSynchronously(bind(&grasp::TmsRpBar::onSimulationInfoButtonClicked,grasp::TmsRpBar::instance()));
				    			sleep(1);
				    		}
						}
					}
				}
				// move joint
				callSynchronously(bind(&TmsRpSubtask::sp5_control,this, type, UNIT_ALL, CMD_CALC_BACKGROUND, 19, goal_arg));

				break;
			}
			case 2006:
			{
				if (voronoi_path_planning_client.call(rp_srv)) {
					os << "result: " << rp_srv.response.success << " message: " << rp_srv.response.message << endl;
					if (rp_srv.response.VoronoiPath.size()!=0) {
						double arg[13];
						grasp::PlanBase *pb = grasp::PlanBase::instance();

						pb->setGraspingState(grasp::PlanBase::GRASPING);
						for (int i=0; i< rp_srv.response.VoronoiPath.size(); i++) {
							arg[0] = rp_srv.response.VoronoiPath[i].x;
							arg[1] = rp_srv.response.VoronoiPath[i].y;
							arg[2] = rp_srv.response.VoronoiPath[i].th;
							tms_msg_rc::tms_rc_pmove kxp_srv;
							kxp_srv.request.w_x = arg[0];
							kxp_srv.request.w_y = arg[1];
							kxp_srv.request.w_th = arg[2];
							if (kxp_mbase_client.call(kxp_srv)) ROS_INFO("result: %d", kxp_srv.response.success);
							else                  ROS_ERROR("Failed to call service kxp_mbase");

							if (type == false) {
								double rPosX = arg[0]/1000;
				    			double rPosY = arg[1]/1000;
				    			double rPosZ = 0;
				    			cnoid::Vector3 rpy (0, 0, deg2rad(arg[2]));
				    			cnoid::Matrix3 rot = grasp::rotFromRpy(rpy);

				    			callSynchronously(bind(&grasp::TmsRpController::disappear,tac,"kxp"));
				    			callSynchronously(bind(&grasp::TmsRpController::setPos,tac,"kxp",cnoid::Vector3(rPosX,rPosY,rPosZ), rot));
							}

				    		pb->bodyItemRobot()->body()->calcForwardKinematics();

				    		cnoid::Vector3 position;
							cnoid::Vector3 rotation;
				    		rotation = grasp::rpyFromRot(pb->fingers(0)->tip->R()*(pb->targetArmFinger->objectPalmRot));
				    		position = pb->fingers(0)->tip->p()+pb->fingers(0)->tip->R()*pb->targetArmFinger->objectPalmPos;
				    		//callSynchronously(bind(&grasp::PlanBase::flush,pb));

							ROS_INFO("robot_x=%f,y=%f,z=%f", arg[0], arg[1], arg[2]);
							ROS_INFO("object_x=%f,y=%f,z=%f,rr=%f,rp=%f,ry=%f\n", position(0), position(1), position(2), rad2deg(rotation(0)), rad2deg(rotation(1)), rad2deg(rotation(2)));

							tms_msg_db::TmsdbGetData name_srv;
							name_srv.request.tmsdb.name = pb->targetObject->bodyItemObject->name();
							if(get_data_client.call(name_srv))
							arg[3] = 2; // robot_state
							arg[4] = (double)name_srv.response.tmsdb[0].id; // obj_id
							arg[5] = position(0)*1000; // m -> mm;
							arg[6] = position(1)*1000;
//							arg[7] = position(2)*1000;
							arg[7] = -1;
							arg[8] = rad2deg(rotation(0));
							arg[9] = rad2deg(rotation(1));
							arg[10] = rad2deg(rotation(2));
							arg[11] = robot_id;
							arg[12] = 2; // obj_state
							callSynchronously(bind(&TmsRpSubtask::kxp_control,this, type, UNIT_ALL, CMD_SYNC_OBJ, 13, arg));

				    		if (type == false) {
				    			sleep(1);
				    			callSynchronously(bind(&grasp::TmsRpBar::onSimulationInfoButtonClicked,grasp::TmsRpBar::instance()));
				    			sleep(1);
				    		}
						}
					}
				}

//				goal_arg[11] = 0.0;
//				goal_arg[12] = 0.0;
//				goal_arg[13] = 0.0;
//				goal_arg[14] = 37.0; // deg
//				goal_arg[15] = 0.0;
				// 関節角遷移
//				callSynchronously(bind(&TmsRpBar::kxp_control,this, type, UNIT_ALL, CMD_CALC_BACKGROUND, 19, goal_arg));

				break;
			}
	    	default:
	    	{
	    		ROS_ERROR("No such robot in TMS_DB!\n");
	    		return false;
	    	}
			}
			if (type == false) {
				sleep(1);
				callSynchronously(bind(&grasp::TmsRpBar::onSimulationInfoButtonClicked,grasp::TmsRpBar::instance()));
				sleep(1);
			}
			//grasp::TmsRpBar::grasping = 0;
			grasp::TmsRpBar::planning_mode = 0;
		    break;
		}
		case 9004: // open the refrigerator
		{
			if (robot_id != 2009) {
				ROS_ERROR("It is an illegal robot_id.\n");
				res.result = 0;
				return false;
			}
			tms_msg_rs::rs_home_appliances ref_srv;
			ref_srv.request.id = robot_id;
			ref_srv.request.service = 1;
			if (refrigerator_client.call(ref_srv)) {}
			else {
			    ROS_ERROR("Failed to call service control_refrigerator");
			    return false;
			}
			break;
		}
		case 9005: // close the refrigerator
		{
			if (robot_id != 2009) {
				ROS_ERROR("It is an illegal robot_id.\n");
				res.result = 0;
				return false;
			}
			tms_msg_rs::rs_home_appliances ref_srv;
			ref_srv.request.id = robot_id;
			ref_srv.request.service = 0;
			if (refrigerator_client.call(ref_srv)) {}
			else {
			    ROS_ERROR("Failed to call service control_refrigerator");
			    return false;
			}
			break;
		}
		default:
		{
			ROS_ERROR("No such subtask (ID : %d)\n", req.command);
			return false;
		}
	}
	res.result = 1;
	return true;
}

// service caller to sp5_(virtual_)control
bool tms_rp::TmsRpSubtask::sp5_control(bool type, int unit, int cmd, int arg_size, double* arg) {
	tms_msg_rc::rc_robot_control sp_control_srv;

	sp_control_srv.request.unit = unit;
	sp_control_srv.request.cmd  = cmd;
	sp_control_srv.request.arg.resize(arg_size);
	for (int i=0; i<sp_control_srv.request.arg.size(); i++) {
		sp_control_srv.request.arg[i] = arg[i];
		ROS_INFO("arg[%d]=%f", i, sp_control_srv.request.arg[i]);
	}

	if (type == true) {
		if (sp5_control_client.call(sp_control_srv)) ROS_INFO("result: %d", sp_control_srv.response.result);
		else                  ROS_ERROR("Failed to call service sp5_control");
	} else {
		if (sp5_virtual_control_client.call(sp_control_srv)) ROS_INFO("result: %d", sp_control_srv.response.result);
		else                  ROS_ERROR("Failed to call service sp5_control");
	}
	return true;
}

// service caller to kxp_(virtual_)control
bool tms_rp::TmsRpSubtask::kxp_control(bool type, int unit, int cmd, int arg_size, double* arg) {
	tms_msg_rc::rc_robot_control kxp_control_srv;

	kxp_control_srv.request.unit = unit;
	kxp_control_srv.request.cmd  = cmd;
	kxp_control_srv.request.arg.resize(arg_size);
	for (int i=0; i<kxp_control_srv.request.arg.size(); i++) {
		kxp_control_srv.request.arg[i] = arg[i];
		ROS_INFO("arg[%d]=%f", i, kxp_control_srv.request.arg[i]);
	}

	if (type == true) {
//		kxp_control_client.call(kxp_control_srv);
//		if (kxp5_control_client.call(kxp_control_srv)) ROS_INFO("result: %d", kxp_control_srv.response.result);
//		else                  ROS_ERROR("Failed to call service kxp_control");
	} else {
		if (kxp_virtual_control_client.call(kxp_control_srv)) ROS_INFO("result: %d", kxp_control_srv.response.result);
		else                  ROS_ERROR("Failed to call service kxp_control");
	}
	return true;
}

//------------------------------------------------------------------------------
tms_rp::TmsRpView* tms_rp::TmsRpView::instance()
{
  static tms_rp::TmsRpView* instance = new tms_rp::TmsRpView();
  return instance;
}

tms_rp::TmsRpView::TmsRpView(): ToolBar("TmsRpView"), tac(*TmsRpController::instance()) {
	static ros::NodeHandle nh2;
	rp_blink_arrow_server  = nh2.advertiseService("rp_arrow", &TmsRpView::blink_arrow, this);

	mat0       <<  1, 0, 0, 0, 1, 0, 0, 0, 1;  //   0
}

tms_rp::TmsRpView::~TmsRpView()
{
}

//------------------------------------------------------------------------------
//for UR m100
bool tms_rp::TmsRpView::blink_arrow(tms_msg_rp::rp_arrow::Request &req,
                           tms_msg_rp::rp_arrow::Response &res)
{
  int32_t id   = req.id;
  int32_t mode = req.mode;
  string arrow_name;

  if (id==20001) {
    arrow_name = "blink_arrow";
  }
  else if (id==20002) {
    arrow_name = "person_marker1";
  }
  else if (id==20003) {
    arrow_name = "person_marker2";
  }
  else if (id==20004) {
    arrow_name = "person_marker3";
  }
  else if (id==20005) {
    arrow_name = "person_marker4";
  }
  else if (id==20006) {
    arrow_name = "person_marker5";
  }
  else {
    res.result = 0;
    return false;
  }

  if (mode==0)
  {
    callLater(bind(&grasp::TmsRpController::disappear,tac,arrow_name));
    res.result = 1;
    return true;
  }
  else if (mode==1)
  {
    callLater(bind(&grasp::TmsRpController::setPos,tac,arrow_name,cnoid::Vector3(req.x/1000,req.y/1000,req.z/1000), mat0));
    callLater(bind(&grasp::TmsRpController::appear,tac,arrow_name));
    res.result = 1;
    return true;
  }
  else if (mode==2)
  {
    callLater(bind(&grasp::TmsRpController::setPos,tac,arrow_name,cnoid::Vector3(req.x/1000,req.y/1000,req.z/1000), mat0));
    for (int32_t i=0; i < 2; i++)
    {
      callLater(bind(&grasp::TmsRpController::appear,tac,arrow_name));
      sleep(1);
      callLater(bind(&grasp::TmsRpController::disappear,tac,arrow_name));
      sleep(1);
    }
    callLater(bind(&grasp::TmsRpController::appear,tac,arrow_name));
    res.result = 1;
    return true;
  }

  res.result = 0;
  return false;
}