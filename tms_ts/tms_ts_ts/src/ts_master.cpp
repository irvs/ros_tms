/// @brief URを受け付けて，
///        tms_ts_nodeletを起動させる.
//コマンドで送る命令↓
//rosrun nodelet nodelet load tms_ts_nodelet/TS nodelet_manager __name:=ts_nodelet1 ts_nodelet1/tms_ts:=req1
//rosrun nodelet nodelet load tms_ts_nodelet/TS nodelet_manager __name:=nodelet1 nodelet1/tms_ts:=req1
#include <tms_ts/ts_master.hpp>

//グローバル変数 リスト
std::list<TmsTsMaster::Task> task_list_;
// リストの先頭要素の値を返す関数
int at_rostime(void) {
	std::list<TmsTsMaster::Task>::iterator it = task_list_.begin();
	return it->rostime;
}
int at_taskID0(void) {
	std::list<TmsTsMaster::Task>::iterator it = task_list_.begin();
	return it->task_id;
}
int at_robotID0(void) {
	std::list<TmsTsMaster::Task>::iterator it = task_list_.begin();
	return it->robot_id;
}
int at_objectID0(void) {
	std::list<TmsTsMaster::Task>::iterator it = task_list_.begin();
	return it->object_id;
}
int at_userID0(void) {
	std::list<TmsTsMaster::Task>::iterator it = task_list_.begin();
	return it->user_id;
}
int at_placeID0(void) {
	std::list<TmsTsMaster::Task>::iterator it = task_list_.begin();
	return it->place_id;
}
int at_priority(void) {
	std::list<TmsTsMaster::Task>::iterator it = task_list_.begin();
	return it->priority;
}

std::string TmsTsMaster::run_nodelet_manager = "rosrun nodelet nodelet manager __name:=nodelet_manager\n";
std::string TmsTsMaster::rosrun = "rosrun nodelet nodelet load tms_ts_nodelet/ROS_TMS_TS nodelet_manager __name:=";
std::string TmsTsMaster::node_name = "ts_nodelet";
std::string TmsTsMaster::service_name = "request";

// コンストラクタ
TmsTsMaster::TmsTsMaster(int argc, char **argv) {
	// task initialize
	for (uint32_t i=0; i<MAX_TASK_NUM*2; i++) {
		task_manager[i].num = boost::lexical_cast<std::string>(i);
		task_manager[i].flag = 0;
//		cout << task_manager[i].num << endl;
//		ROS_INFO("flag=[%u]\n", task_manager[i].flag);
	}

	// ros init
	ros::init(argc, argv, "tms_ts_master");
	ros::NodeHandle n;
	service = n.advertiseService("tms_ts_master", &TmsTsMaster::ts_master_callback, this);
}

// thread1用 rosspin
void TmsTsMaster::ros_spin() {
	while(ros::ok()) {
	ros::spin();
	}
}

std::string TmsTsMaster::CreateSrvCall(long long int rostime, int task_id, int robot_id, int object_id,
		int user_id, int place_id, int priority, int thread_num) {
	//std::string s_rostime = boost::lexical_cast<std::string>(rostime);
	std::string s_rostime("0"); // 応急処置
	//string s_task_id = IntToString(task_id);
	std::string s_task_id = boost::lexical_cast<std::string>(task_id);
	std::string s_robot_id = boost::lexical_cast<std::string>(robot_id);
	std::string s_object_id = boost::lexical_cast<std::string>(object_id);
	std::string s_user_id = boost::lexical_cast<std::string>(user_id);
	std::string s_place_id = boost::lexical_cast<std::string>(place_id);
	std::string s_priority = boost::lexical_cast<std::string>(priority);

	//rosservice call /request{num} "rostime" "task_id" "robot_id" "object_id" "user_id" "place_id" "priority"\n
	std::string command("rosservice call /");
	command += service_name;
	command += task_manager[thread_num].num;
	command += " \"";
	command += s_rostime;
	command += "\" \"";
	command += s_task_id;
	command += "\" \"";
	command += s_robot_id;
	command += "\" \"";
	command += s_object_id;
	command += "\" \"";
	command += s_user_id;
	command += "\" \"";
	command += s_place_id;
	command += "\" \"";
	command += s_priority;
	command += "\"\n";

	return command;
}

std::string TmsTsMaster::CreateRunCmd(int thread_num) {
	std::string command;
	std::string space(" ");
	std::string enter("\n");

	command = rosrun+node_name;
	command += task_manager[thread_num].num;
	command += space;
	command += node_name;
	command += task_manager[thread_num].num;
	command += "/tms_ts:=";
	command += service_name;
	command += task_manager[thread_num].num;
	command += enter;
	//ROS_INFO("%s\n", command.c_str());
	return command;
}

bool TmsTsMaster::ExecuteCmd(const char* buf) {
//	mutex::scoped_lock lk(mtx); // このスコープは1つのスレッドしか入れない
	int ret;
	std::cout << buf << std::endl;//サブタスクの内容を表示

//	ret = system("buf >> /home/hashiguchi/test.txt");
	ret = system(buf);
	// コマンドが実行できなかった場合
	if(ret != 0) {
		ROS_ERROR("Shut down system call\n");
		return false;
	}
	return true;
}

//string TmsTsMaster::IntToString(int number) {
//	ostringstream oss;
//	oss << number;
//	return oss.str();
//}

// ここではタスクリストにrequestデータを入れるだけ
bool TmsTsMaster::ts_master_callback(tms_msg_ts::ts_req::Request &req,
		tms_msg_ts::ts_req::Response &res)
{
	ROS_INFO("in callback function.\n");

	ros::Time time_now = ros::Time::now();
	Task tmp_task;
	tmp_task.task_id = req.task_id;
	tmp_task.robot_id = req.robot_id;
	tmp_task.object_id = req.object_id;
	tmp_task.user_id = req.user_id;
	tmp_task.place_id = req.place_id;
	tmp_task.priority = req.priority;
	tmp_task.rostime = time_now.toNSec();
	task_list_.push_back(tmp_task);

	// // 第1条件：優先度の高い順,第2条件：rostimeの早い順でソート
	task_list_.sort(pred());
	std::list<Task>::iterator it = task_list_.begin(); // イテレータ
	while( it != task_list_.end() )  // listの末尾まで
	{
		std::cout << it->task_id << std::endl;  // task_idを出力
		std::cout << it->priority << std::endl;  // priorityを出力
		std::cout << it->rostime << std::endl;  // timeを出力
		++it;  // イテレータを１つ進める
	}
	//	ExecuteCmd((CreateSrvCall(req.task_id, req.object_id)).c_str());
	res.result = 1; // success
	return true;
}

int main(int argc, char **argv)
{
	task_list_.clear();
	TmsTsMaster ts(argc, argv); // クラスの実体をつくる

	static std::string manager = ts.run_nodelet_manager;
//	string ts_clone1 = ts.CreateCmd(0);
//	string ts_clone2 = ts.CreateCmd(1);

	//nodelet manager起動
	boost::thread thread0(boost::bind(&TmsTsMaster::ExecuteCmd, &ts, manager.c_str()));
	//service用spin
	boost::thread thread1(boost::bind(&TmsTsMaster::ros_spin, &ts));
	// task実行用スレッドの配列
	boost::thread *threads[MAX_TASK_NUM*2]; // 並列スレッド,task_managerと対応

	ros::Rate loop_rate(1); // 1Hz
	// タスクが入っていたら新しいスレッドで実行(1秒間隔で確認)
	while(ros::ok()) {
		if (task_list_.empty() == 1) { // 要求タスクなし
		} else { // 要求タスクあり
			for (int i=0; i<MAX_TASK_NUM; i++) {
				if (ts.task_manager[i].flag == 0) { // 空きスレッド
					ts.task_manager[i].flag = 1;
					ROS_INFO("Thread number is %d", i);
					// タスク情報格納後,リストから削除
					std::string srv_req = ts.CreateSrvCall(at_rostime(), at_taskID0(), at_robotID0(), at_objectID0(), at_userID0(), at_placeID0(), at_priority(), i);
					//cout << srv_req.c_str() << endl;
					task_list_.pop_front();
					// threads[i]でnodelet node立ち上げ
					threads[i] = new boost::thread(boost::bind(&TmsTsMaster::ExecuteCmd, &ts,
							(ts.CreateRunCmd(i)).c_str()));
					// !!!!!!!ExecuteCmd関数をthread排他制御したいがsystem関数のため並列化できなくなる
					ros::Duration(3.0).sleep();
					// 立ち上げたnodelet nodeに対してthreads[i+10]からservice call
					threads[i+10] = new boost::thread(boost::bind(&TmsTsMaster::ExecuteCmd, &ts, srv_req.c_str()));
					break;
					} else if (i == MAX_TASK_NUM-1) {
						ROS_WARN("No free-thread! Sleep for 3.0 seconds.\n");
						ros::Duration(3.0).sleep();
						i = 0; // repeat
					}
				}
			}
		loop_rate.sleep();
	}
	return 0;
}
