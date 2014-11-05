/*
 * ts_master.hpp
 *
 *  Created on: 2013/10/29
 *      Author: hashiguchi
 */

#ifndef TS_MASTER_HPP_
#define TS_MASTER_HPP_

#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <string.h>
#include <vector>
#include <list>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <tms_msg_ts/ts_req.h>

//using namespace std;
//using namespace boost;

#define MAX_TASK_NUM 30

class TmsTsMaster {
public:
	// コマンド生成用
	static std::string run_nodelet_manager;
	static std::string rosrun;
	static std::string node_name;
	static std::string service_name;

	TmsTsMaster(int argc, char **argv);
	void ros_spin(void);
	std::string CreateSrvCall(long long int rostime, int task_id, int robot_id, int object_id,
			int user_id, int place_id, int priority, int thread_num);
	std::string CreateRunCmd(int thread_num);
	bool ExecuteCmd(const char* buf);

	// TMS TASK
	struct Task {
		long long int rostime;
		int task_id;
		int robot_id;
		int object_id;
		int user_id;
		int place_id;
//		// 条件
		int priority; // 優先度,緊急性を表す
//		long long int time; // 定時タスク用
//		int dependence;  // タスク依存関係
	};

	struct TaskManager {
//		static string num;
		std::string num;
		bool flag;
	};
//	list<Task> task_list_;
	TaskManager task_manager[MAX_TASK_NUM*2];
private:
	// ファンクタを作ってソート
	struct pred {
	    bool operator()(const Task& l, const Task& r) const {
	    	return l.priority == r.priority ? l.rostime < r.rostime : l.priority < r.priority;
	    }
	};

	//std::string IntToString(int number);
	bool ts_master_callback(tms_msg_ts::ts_req::Request &req,
			tms_msg_ts::ts_req::Response &res);

	boost::mutex mtx; // スレッドの排他制御用
	ros::ServiceServer service;

};

#endif /* TS_MASTER_HPP_ */
