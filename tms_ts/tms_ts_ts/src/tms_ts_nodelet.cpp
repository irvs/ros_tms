#include <tms_ts/tms_ts_nodelet.hpp>

void tms_ts_nodelet::ROS_TMS_TS::onInit() {
	// vector配列初期化
	sub_task.clear();
	file_name = "";

	ros::NodeHandle nh;
	ros::NodeHandle& private_nh = getPrivateNodeHandle();
	service = private_nh.advertiseService("tms_ts", &ROS_TMS_TS::tsCallback, this);
	// for connecting to ROSTMS_DB task,subtask
	db_reader_client = nh.serviceClient<tms_msg_db::TmsdbGetData>("tms_db_reader/dbreader");
	db_reader_client2 = nh.serviceClient<tms_msg_db::TmsdbGetData>("tms_db_reader/dbreader");
}

std::string tms_ts_nodelet::ROS_TMS_TS::rosCheckTime(boost::posix_time::ptime time)
{
    struct tm T = boost::posix_time::to_tm(time);     // ptime -> struct tm
    std::ostringstream oss;

    oss << T.tm_year +1900 << T.tm_mon + 1 << T.tm_mday << "_" << T.tm_hour << T.tm_min << T.tm_sec;
    return oss.str();
}

void tms_ts_nodelet::ROS_TMS_TS::GenerateScript() {
	// ファイル名生成(time)
	std::ostringstream oss_filename;
	oss_filename.str(""); // initialize

	// 現在の日時を取得
	ros::Time tNow = ros::Time::now() + ros::Duration(9*60*60); // GMT +9
	oss_filename << ROS_TMS_TS::rosCheckTime(tNow.toBoost()) << ".py";
	ROS_TMS_TS::file_name = oss_filename.str();

	char home_dir[255];
	strcpy(home_dir, getenv("HOME"));
	std::string file_name_full = home_dir;
	file_name_full += "/catkin_ws/src/ros_tms/tms_ts/tms_ts_smach/scripts/";
	file_name_full +=  ROS_TMS_TS::file_name;

	// 出力ファイルが既に存在するとエラー
	std::ofstream ofs(file_name_full.c_str(), std::ios::in);
	if (ofs) {
		// error, file exists!
		ROS_ERROR("There is a file named \'%s\'", file_name_full.c_str());
		exit(1);
	}
	else
	{
	    ofs.close();
	    ofs.open(file_name_full.c_str(), std::ios::out | std::ios::ate);  // OK now
	    if (!ofs) {
	    	ROS_ERROR("***error  Cannot open this file!\n");
	    	exit(1);
	    }

	    // =======================================================================
	    // =========================pythonスクリプト生成 =========================
	    // =======================================================================
	    ofs << "#!/usr/bin/env python\n\nimport roslib; roslib.load_manifest('tms_ts_smach')\n"
	    		"import rospy\nimport smach\nimport smach_ros\n\nfrom smach_ros import ServiceState\n"
	    		"from tms_msg_rp.srv import *\n\n" << std::endl;

	    std::ostringstream state_name;
	    std::ostringstream next_state_name;
	    std::ostringstream command;
	    std::ostringstream argument;

	    // DEFINE STATE
	    for (int i = 0; i < ROS_TMS_TS::sub_task.size(); i++) {
	    	// 状態の名前定義
	    	state_name.str(""); // 初期化
	    	state_name << ROS_TMS_TS::sub_task.at(i).subtask_name << i;
	    	// ファイル出力
	    	ofs << "class " << state_name.str() << "(smach.State):\n    def __init__(self):\n"
	    			"        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])\n\n"
	    			"    def execute(self, userdata):\n        rospy.loginfo('Executing state " << state_name.str() << "')\n\n" << std::endl;
	    }

	    // MAIN
	    ofs << "def main():\n    rospy.init_node('tms_ts_smach_test')\n\n"
	    		"    sm_root = smach.StateMachine(['succeeded','aborted','preempted'])\n\n"
	    		"    with sm_root:\n" << std::endl;

	    // TRANSITION
	      // SERVICE CALL : rosservice call /rp_cmd "command" "robot_id" "[argument]"\n
	    for (int i = 0; i < ROS_TMS_TS::sub_task.size(); i++) {
	    	// 状態名定義
	    	state_name.str(""); // 初期化
	    	state_name << ROS_TMS_TS::sub_task.at(i).subtask_name << i;

	    	// サブタスクID定義
	    	command.str("");
	    	command << ROS_TMS_TS::sub_task.at(i).subtask_id;
	    	ofs << "        smach.StateMachine.add('" << state_name.str() << "',\n                           ServiceState('rp_cmd',\n"
	    			"                                        rp_cmd,\n                                        request = rp_cmdRequest("
	    			<< command.str() << ", False, " << ROS_TMS_TS::robot_id << ", [";
//	    			<< command.str() << ", " << ROS_TMS_TS::robot_id << ", " << ROS_TMS_TS::sub_task.at(i).arg << ")),\n"; // 引数生成
	    	int check = 0;
	    	for (int j = 0; j < ROS_TMS_TS::sub_task.at(i).arg.size(); j++) {
	    		if ( check >= 1) {
	    			ofs << ", ";
	    		}
	    		argument.str("");
	    		argument << ROS_TMS_TS::sub_task.at(i).arg.at(j);
	    		ofs << argument.str();
	    		check++;
	    	}
	    	ofs << "])),\n";
	    	// 最終状態
	    	if (i == ROS_TMS_TS::sub_task.size() - 1) {
	    	    ofs << "                           transitions={'succeeded':'succeeded'})\n" << std::endl;
	    	} else {
	    		next_state_name.str(""); // 初期化
		    	next_state_name << ROS_TMS_TS::sub_task.at(i+1).subtask_name << i+1;
		    	ofs << "                           transitions={'succeeded':'" << next_state_name.str() << "'})\n" << std::endl;
	    	}
	    }

	    // FOR SMACH VIEWER
	    ofs << "    sis = smach_ros.IntrospectionServer('server_name', sm_root, '/SM_ROOT')\n"
	    		"    sis.start()\n\n    outcome = sm_root.execute()\n\n    rospy.spin()\n"
	    		"    sis.stop()\n\nif __name__ == '__main__':\n    main()" << std::endl;

	    ofs.close();
	}
    ROS_INFO("Finish generating python script to tms_ts_smach/scripts/%s\n", ROS_TMS_TS::file_name.c_str());
}

//  pythonスクリプトの実行
bool tms_ts_nodelet::ROS_TMS_TS::ExeScript() {
	int ret;

	std::string chmod(""); // フォルダ移動 & 権限付与
	std::string command(""); // 実行命令

	chmod = "cd ~/catkin_ws/src/ros_tms/tms_ts/tms_ts_smach && chmod +x scripts/" + ROS_TMS_TS::file_name;
	command = "cd ~/catkin_ws/src/ros_tms/tms_ts/tms_ts_smach && ./scripts/" + ROS_TMS_TS::file_name;

	const char *buf1 = chmod.c_str();
	const char *buf2 = command.c_str();
	ROS_INFO("CMD1:%s\n", buf1);
	ROS_INFO("CMD2:%s\n", buf2);

	ret = system(buf1);
	if(ret != 0) {
		ROS_ERROR("Execute chmod error\n");
		return false;
	}

	ros::Duration(0.5).sleep(); //temp

	ret = system(buf2);
	if(ret != 0) {
		ROS_ERROR("Execute script error\n");
		return false;
	}

	return true;
}

bool tms_ts_nodelet::ROS_TMS_TS::tsCallback(tms_msg_ts::ts_req::Request &req, tms_msg_ts::ts_req::Response &res) {

	int sid = 100000;
	// リクエストされたidを保持
	robot_id = req.robot_id;
	object_id = req.object_id;
	user_id = req.user_id;
	place_id = req.place_id;

// requestと一致するタスクidを持つタスク情報をrostms_db idテーブルから取得
	tms_msg_db::TmsdbGetData srv;
	tms_msg_db::TmsdbGetData srv2;
	srv.request.tmsdb.id = req.task_id + sid;

	if(db_reader_client.call(srv)) {
		ROS_INFO("get task data from tms_db\n");

	    std::string subtask = srv.response.tmsdb[0].etcdata;
	    std::vector<std::string> seq_of_subtask;

	    // initialize
	    seq_of_subtask.clear();
	    boost::split(seq_of_subtask, subtask, boost::is_any_of(";"));

	    for (int cnt = 0; cnt < seq_of_subtask.size(); cnt++) {
	    	ROS_TMS_TS::SubTask st;
//	    	st.arg.clear(); // initialize

	    	if (cnt % 2 == 0) { // 偶数 subtaskID
		    	st.subtask_id = seq_of_subtask.at(cnt);
		    	ROS_INFO("subtask_id = [%s]\n", seq_of_subtask.at(cnt).c_str());
	    		srv2.request.tmsdb.id = std::atoi(seq_of_subtask.at(cnt).c_str()) + sid;

		    	if(db_reader_client2.call(srv2)) {
		    		st.subtask_name = srv2.response.tmsdb[0].name;
	//		    		std::string argument = srv2.response.tmsdb[0].etcdata;
	//		    		std::vector<std::string> arg;
	//		    		arg.clear();
	//		    		boost::split(arg, argument, boost::is_any_of(";"));
	//		    		for (int cnt2 = 0; cnt2 < arg.size(); cnt2++) {
	//		    			ROS_INFO("subtask[%s]'s arg[%d] = [%s]\n", seq_of_subtask.at(cnt).c_str(), cnt2, arg.at(cnt2).c_str());
	//		    			st.arg.push_back(arg.at(cnt2));
	//		    		}

		    		ROS_INFO("arg = %s", seq_of_subtask.at(cnt + 1).c_str());
//		    		ROS_INFO("arg type = %s", seq_of_subtask.at(cnt + 1).substr(0, 3).c_str());
//		    		ROS_INFO("pose type = %s", seq_of_subtask.at(cnt + 1).substr(3, seq_of_subtask.at(cnt + 1).size()).c_str());

		    		// argumentの代入
		    		std::vector<std::string> seq_of_argument;
		    		seq_of_argument.clear();
		    		boost::trim_if (seq_of_subtask.at(cnt + 1), boost::is_any_of("$") );
		    		boost::split(seq_of_argument, seq_of_subtask.at(cnt + 1), boost::is_any_of("$"));

		    		for (int i=0; i<seq_of_argument.size(); i++) {
			    		if (seq_of_argument.at(i) == "oid") { // object_id
			    			st.arg.push_back(ROS_TMS_TS::object_id);
			    		} else if (seq_of_argument.at(i) == "uid") { // user_id
			    			st.arg.push_back(ROS_TMS_TS::user_id);
			    		} else if (seq_of_argument.at(i) == "pid") { // place_id
			    			st.arg.push_back(ROS_TMS_TS::place_id);
			    		} else if (seq_of_argument.at(i) == "rid") { // robot_id
			    			st.arg.push_back(ROS_TMS_TS::robot_id);
//			    		} else if (seq_of_argument.at(i).substr(0, 2) == "p_") { // pose_id
//			    			// 特定の姿勢に関するデータの取得先は要検討 ?pose database?
//			    			if (seq_of_subtask.at(cnt + 1).substr(3, seq_of_argument.at(i).size() - 1) == "take") {
//			    				st.arg.push_back(11001);
//			    			} else if (seq_of_subtask.at(cnt + 1).substr(3, seq_of_subtask.at(cnt + 1).size()) == "give") {
//			    				st.arg.push_back(11002);
//			    			} else if (seq_of_subtask.at(cnt + 1).substr(3, seq_of_subtask.at(cnt + 1).size()) == "init") {
//			    				st.arg.push_back(11003);
//			    			} else {
//			    				ROS_ERROR("No such argument p_*\n");
//			    				return 0;
//			    			}
			    		} else {
			    			ROS_ERROR("No such argument!\n");
			    			return 0;
			    		}
		    		}
		    		ROS_TMS_TS::sub_task.push_back(st);
		    	} else {
		    		ROS_INFO("Failed to call service tms_db_get_subtask_data.\n");
		    		res.result = 0; // false1
		    		return true;
		    	}
	    	}
	    }
	}else { //TMSDBからtask_listsの取得に失敗したとき
		ROS_INFO("Failed to call service tms_db_get_task_data.\n");
		res.result = 0; // false1
		return true;
	}

	ROS_TMS_TS::GenerateScript();
	ROS_TMS_TS::ExeScript();

	res.result = 1; // success
	return true;
}
