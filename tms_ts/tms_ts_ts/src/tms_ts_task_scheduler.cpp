/*task_scheduler

analyze part から送られてきたタスクについて

・どのロボットに割り振るか？
・どのタスクを優先するか？

を確定させ，実際にタスクをロボットに行わせる。

・どのロボットが、どのタスクを実行しているか？

を管理する必要がある。
※タスク実行中の機体の確認はDBで行う？

パターン1: リスト形式？でタスクを保持し、一定時間毎にスケジュールし直す
パターン2: マルチスレッドループを使う？

*/

//include for ROS
#include "ros/ros.h"
#include <unistd.h>
#include <boost/date_time/local_time/local_time.hpp>

//#include "tmsdb/tmsdb_get_objects_info.h"
//#include "tmsdb/tmsdb_get_person_info.h"
#include <tms_msg_db/tmsdb_get_person_behavior_info.h>

#include <tms_msg_sa/tms_to_activate_service.h>
#include <tms_msg_ts/tms_sa_find_objects.h>
#include <tms_msg_rp/tms_ts_find_objects.h>

#include <tms_msg_sa/tms_to_modify_missing_objects.h>

ros::ServiceServer service01;
//ros::ServiceServer service02;

ros::ServiceServer service11;
//ros::ServiceServer service12;

//client -> tmsdb
ros::ServiceClient client01;

ros::ServiceClient client1;
//ros::ServiceClient client2;
//ros::ServiceClient client3;
//ros::ServiceClient client4;
//ros::ServiceClient client5;


tms_msg_ts::tms_sa_find_objects::Request task_find_objects;

bool tms_utility_activate_task_func(tms_msg_sa::tms_to_activate_service::Request &req, tms_msg_sa::tms_to_activate_service::Response &res){

	return true;
}

bool tms_utility_excommand_1_func(tms_msg_sa::tms_to_activate_service::Request &req, tms_msg_sa::tms_to_activate_service::Response &res){

	//TaskControllerとの調整必要
	// srv1;
	// srv1.~=
	//client.call
	//...

	res.success = 1;
	return true;
}

//物品探索タスクをタスクリストに保持
//タスクが増えた場合，優先度・有効フラグを用いてタスクを管理する
bool tms_sa_find_objects_TS_func(tms_msg_ts::tms_sa_find_objects::Request &req, tms_msg_ts::tms_sa_find_objects::Response &res){

	tms_msg_rp::tms_ts_find_objects srv1;

	ROS_INFO("tms_sa_find_objects_ts_func");

	srv1.request.search_furnitures_id = req.search_furnitures_id;

	//最新のfind_objectsの状態を保存する.
	task_find_objects = req;

	res.message = "success : task_find_objects task information update";

	return true;
}

//人の状態：退室の分析とタスクの実行
void callback1(const ros::TimerEvent&){

	tms_msg_db::tmsdb_get_person_behavior_info srv01;

	tms_msg_ts::tms_sa_find_objects srv1;

	//動作条件に当たるかどうか(人の現在の状態=退室？)を判定する
	if(client01.call(srv01)){
		ROS_INFO("%u, %u", srv01.response.id, srv01.response.behavior);
	}
	else{
		ROS_ERROR("Failed to call service tmsdb_get_person_behavior");
	}

	if(srv01.response.behavior == 0){
		ROS_INFO("****************** Trigger ******************");

		printf("task_find_objects : ");
		for(unsigned int i=0;i<task_find_objects.search_furnitures_id.size();i++){
			printf("[%d]", task_find_objects.search_furnitures_id[i]);
		}
		printf("\n");

		//最新のサービス引数を代入する
		srv1.request = task_find_objects;

		if(client1.call(srv1)){
			if(srv1.response.message.empty() != true)
				ROS_INFO("task_message : %s",srv1.response.message.c_str());

			//タスクの結果、object_idとfurnitures_idの組が返ってきたのであれば
			//情報をmissing_objects_listから引き出してきて、writerへ投げて書き込む
			if(srv1.response.object_id.empty() != true){
				for(unsigned int i=0;i<srv1.response.object_id.size();i++){
					ROS_INFO("object_id = %d, place = %d\n", srv1.response.object_id[i], srv1.response.furnitures_id[i]);
				}
			}
			else{
				ROS_INFO("NOT FIND OBJECTS");
			}
		}
		else{
			ROS_ERROR("Failed to call service tms_ts_find_objects");
		}


	}
	else{
		ROS_INFO("behavior != 0");
		for(unsigned int i=0;i<task_find_objects.search_furnitures_id.size();i++){
			printf("[%d]", task_find_objects.search_furnitures_id[i]);
		}
		printf("\n");
	}

}

int main(int argc, char **argv){

	ros::init(argc, argv, "tms_ts_task_scheduler");
	ros::NodeHandle n;
	ROS_INFO("tms_ts_task_scheduler : init");

	//multithreadspinner
	ros::MultiThreadedSpinner spinner(2);

	//スケジューラからの起動の場合
	ros::Timer timer1 = n.createTimer(ros::Duration(30.0), callback1);


//	特殊サービス:service0
//	これを出すと,命令実行パートへ移る?
//	service0 = n.advertiseService("tms_utility_activate_task", tms_utility_activate_task_func);


//	service : from analyze part task(0X) or external input task(1X)
	service01 = n.advertiseService("tms_sa_find_objects_TS", tms_sa_find_objects_TS_func);
//	service02 = n.advertiseService("tms_utility_task_...", tms_analyze_..._func);
//	service03 = n.advertiseService("tms_utility_task_...", tms_analyze_..._func);
//	service04 = n.advertiseService("tms_utility_task_...", tms_analyze_..._func);
//	...
	service11 = n.advertiseService("tms_utility_excommand_1", tms_utility_excommand_1_func);
//	service12 = n.n.advertiseService("tms_utility_excommand_2", tms_utility_excommand_2_func);
//	...

//	client : to task controller part
	client1 = n.serviceClient<tms_msg_rp::tms_ts_find_objects>("tms_ts_find_objects");
//	client2 = n.serviceClient<tms_utility::tms_utility_task_...>("tms_utility_task_...");
//	client3 = n.serviceClient<tms_utility::tms_utility_task_...>("tms_utility_task_...");
//	client4 = n.serviceClient<tms_utility::tms_utility_task_...>("tms_utility_task_...");
//	client5 = n.serviceClient<tms_utility::tms_utility_task_...>("tms_utility_task_...");
//	...

	client01 = n.serviceClient<tms_msg_db::tmsdb_get_person_behavior_info>("tmsdb_get_person_info_3");

	ROS_INFO("tms_ts_task_scheduler : wait");

	//Multi Thread Spinner：
	//1. 各分析パートからのタスク.Requestを保存しつづける．(分析の数だけspinner要素が必要)
	//2. 定期的に部屋内の状況が特定タスクの起動条件に当たるかどうかを見る.(ロボットサービスの数だけspinner要素が必要)
	//ros::spin();
	spinner.spin();

}
