//include for ROS
#include "ros/ros.h"
#include <unistd.h>
#include <boost/date_time/local_time/local_time.hpp>

#include "tms_msg_ss/fss_object_data.h"
#include "tms_msg_ss/ics_object_data.h"
#include "tms_msg_rc/tag_data.h"


//#include "tmsdb/tmsdb_sche_to_find.h"
//#include "tmsdb/tmsdb_modify_person_behavior.h"
//#include "tmsdb/tmsdb_modify_missing_objects.h"

#include "tms_msg_ss/fss_person_trajectory_data.h"
#include "tms_msg_sa/bas_behavior_data.h"
#include "tms_msg_ss/fss_observed_datas.h"
#include "tms_msg_db/tmsdb_data.h"

#include "tms_msg_db/tmsdb_ods_object_data.h"
#include "tms_msg_db/tmsdb_get_objects_info.h"

#include <tms_msg_rc/robot_current_data.h>

//include srv.h
#include "tms_msg_db/tmsdb_file_conservation.h"
#include "tms_msg_db/tmsdb_objects_data.h"

//include for MySQL
#include <stdio.h>
#include <stdlib.h>
#include <mysql/mysql.h>
#include <sstream>
#include <string>

//MySQL login Macro
#define DBHOST "192.168.4.170"
#define DBUSER "root"
#define DBPASS "tmsdb"
#define DBNAME "tmsdb"

//include for PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>


//#include <capture.h>

//#define DBHOST "133.5.20.38"
//#define DBUSER "user13"
//#define DBPASS "BF2Ax7H4"
//#define DBNAME "tmsdb"

MYSQL *conn;	//MySQL Connector
MYSQL_RES *result;
MYSQL_ROW row;

FILE *fp;

#define MAX_OBJECT_ICS 100
#define START_OBJECTS_ID 51

struct BChecker{
	int id_b;
	int year_b;
	int month_b;
	int day_b;
	int hour_b;
	int minute_b;
	int second_b;
	float x_b;
	float y_b;
	float th_b;
};

struct BChecker_ics{
//	int id_b;
//	int year_b;
//	int month_b;
//	int day_b;
//	int hour_b;
//	int minute_b;
//	int second_b;
	float x_b;
	float y_b;
	float z_b;
//	float th_b;
//	float weight_b;
//	float place_b;
};

struct Vanish_ics{
	int vanish_flag;
	int vanish_id;
//	std::string ics_name;
};

struct Ics_before_state{
	int iID;
	float x;
	float y;
	float weight;
	int iState;
};

BChecker bchecker[1000];	//maxid = 999
BChecker_ics bchecker_ics[MAX_OBJECT_ICS]; //maxid = 999
Vanish_ics vanish[256];	//-vanish_flag: 各IDの物品が存在するか　存在したらフラグ1
							//-vanish_id: 各ID
std::vector<Ics_before_state> ics_before_state;	//1つ前のICSからのstate情報を保持する
bool ics_before_state_F_flag = true;
unsigned int ics_before_size;

//for person tracking
//unsigned int person_tracking_array_num;	//fss_trajectory_datas[person_tracking_array_num]から入れる
//unsigned int b_trajectory_size;
//unsigned int b_fcenterx_size;
//long long unsigned int b_person_track_time;
float b_person_x;
float b_person_y;
long long unsigned int b_person_time;
std::vector<tms_msg_db::tmsdb_data> b_msgTMSInfo_person;
tms_msg_sa::bas_behavior_data b_bas_behavior_data;

//for robot tracking
long long unsigned int DBwriteCallback_select[256];

//for movable_furnitures tracking
float DBwriteCallback_m_furnitures_x[256];
float DBwriteCallback_m_furnitures_y[256];


//ロボット(Type=1)、移動家具(Type=3)移動履歴
//tmsdb.robots_historyへ書き込み(INSERT)
//tmsdb.robots(NOW)は更新(UPDATE)、存在しない場合のみ書き込み(INSERT)
void DBwriteCallback(const tms_msg_ss::fss_object_data::ConstPtr& msg){


    ROS_INFO("in db write callback");

	//query to MySQL

	//UPDATE用クエリ
	char query1[1024];
	//INSERT用クエリ
	char query2[1024];

	//情報が入っている
	if(msg->msgTMSInfo.empty() == false){

		//<--for demo 12.01.28 
		//UPDATE process
		/*
		if(msg->msgTMSInfo.front().iType == 1){
            sprintf(query1, "UPDATE robots_present AS A SET A.rostime=%lu,A.x=%f,A.y=%f,A.z=%f,A.theta=%f,A.state=%d,A.place=%d WHERE id=%d;",
					msg->msgTMSInfo[0].tMeasuredTime.toNSec(), msg->msgTMSInfo[0].fX, msg->msgTMSInfo[0].fY, msg->msgTMSInfo[0].fZ,
					msg->msgTMSInfo[0].fTheta, msg->msgTMSInfo[0].iState, msg->msgTMSInfo[0].iPlace, msg->msgTMSInfo[0].iID);

		if (mysql_query(conn, query1)) {
				fprintf(stderr, "%s\n", mysql_error(conn));
				printf("Write error!\n");
			}
		}
		else 
		-->*/
		if(msg->msgTMSInfo.front().iType == 3){
            sprintf(query1, "UPDATE movable_furnitures_present AS A SET A.rostime=%lu,A.x=%f,A.y=%f,A.z=%f,A.theta=%f,A.state=%d,A.place=%d WHERE id=%d;",
					msg->msgTMSInfo[0].tMeasuredTime.toNSec(), msg->msgTMSInfo[0].fX, msg->msgTMSInfo[0].fY, msg->msgTMSInfo[0].fZ,
					msg->msgTMSInfo[0].fTheta, msg->msgTMSInfo[0].iState, msg->msgTMSInfo[0].iPlace, msg->msgTMSInfo[0].iID);

			if (mysql_query(conn, query1)) {
				fprintf(stderr, "%s\n", mysql_error(conn));
				printf("Write error!\n");
			}
		}
		else{
			ROS_INFO("msg TYPE is wrong : UPDATE PROCESS");
		}

		//INSERT process
		//iStateが0のときは取得不可能な場合を示している
		if(msg->msgTMSInfo[0].iState != 0){
			ros::Time tMeasuredTime = msg->msgTMSInfo[0].tMeasuredTime;
			uint64_t nsec = tMeasuredTime.toNSec();

			//正確には各移動体に対して以前に書いた時刻が1秒よりも前であれば書くという条件にする必要がある
			if((DBwriteCallback_select[msg->msgTMSInfo[0].iID]+(1000*1000*1000)) <= nsec){

				//以前に記録した時刻(rostime)を保持
				DBwriteCallback_select[msg->msgTMSInfo[0].iID] = nsec;

				//iType == 1 すなわちrobot_historyへの書き込み
				if(msg->msgTMSInfo[0].iType == 1){
                    sprintf(query2, "INSERT INTO tmsdb.robots_history(rostime,id,x,y,z,theta,state,place) VALUES (%lu,%d,%f,%f,%f,%f,%d,%d);",
							nsec, msg->msgTMSInfo[0].iID, msg->msgTMSInfo[0].fX, msg->msgTMSInfo[0].fY, msg->msgTMSInfo[0].fZ,
							msg->msgTMSInfo[0].fTheta, msg->msgTMSInfo[0].iState, msg->msgTMSInfo[0].iPlace);
//					printf("query2 = %s\n", query2);
//					送られてきた情報を書き込む
//					if (mysql_query(conn, query2)) {
//						fprintf(stderr, "%s\n", mysql_error(conn));
//						printf("Write error!\n");
//					}
				}
				//iType == 3 すなわちmovable_furnitures_historyへの書き込み
				else if(msg->msgTMSInfo[0].iType == 3){
					//以前のmovable_furnitures[id]のx座標と今の座標の差が50mm以上であれば書き込む
					if(sqrt(
							pow(msg->msgTMSInfo[0].fX - DBwriteCallback_m_furnitures_x[msg->msgTMSInfo[0].iID],2) +
							pow(msg->msgTMSInfo[0].fY - DBwriteCallback_m_furnitures_y[msg->msgTMSInfo[0].iID],2)
							                                                       ) > 50){
//						printf("movable_furnitures_t = %f\n", sqrt(
//								pow(msg->msgTMSInfo[0].fX - DBwriteCallback_m_furnitures_x[msg->msgTMSInfo[0].iID],2) +
//								pow(msg->msgTMSInfo[0].fY - DBwriteCallback_m_furnitures_y[msg->msgTMSInfo[0].iID],2)
//								                                                       ));
                        sprintf(query2, "INSERT INTO tmsdb.movable_furnitures_history(rostime,id,x,y,z,theta,state,place) VALUES (%lu,%d,%f,%f,%f,%f,%d,%d);",
								nsec, msg->msgTMSInfo[0].iID, msg->msgTMSInfo[0].fX, msg->msgTMSInfo[0].fY, msg->msgTMSInfo[0].fZ,
								msg->msgTMSInfo[0].fTheta, msg->msgTMSInfo[0].iState, msg->msgTMSInfo[0].iPlace);
//						printf("query2 = %s\n", query2);
						//送られてきた情報を書き込む
						if (mysql_query(conn, query2)) {
							fprintf(stderr, "%s\n", mysql_error(conn));
							printf("Write error!\n");
						}
						//前の座標を更新する
						DBwriteCallback_m_furnitures_x[msg->msgTMSInfo[0].iID] = msg->msgTMSInfo[0].fX;
						DBwriteCallback_m_furnitures_y[msg->msgTMSInfo[0].iID] = msg->msgTMSInfo[0].fY;
					}
				}
				else{
					ROS_INFO("msg TYPE is wrong : INSERT PROCESS");
				}
			}
		}
		else{
			//printf("msg->msgTMSInfo[0].iState == 0\n");
		}
	}
}

//物品挿入・取り出し履歴
//送信側の特徴を利用して簡易的に実装する
//例えば1つずつ出し入れする制約等
void DBwriteCallback_object(const tms_msg_ss::ics_object_data::ConstPtr& msg){

	//INSERT query
	char query1[1024];
	//UPDATE query
	char query2[1024];
	Ics_before_state temp_ics_before_state;


	//メッセージがemptyでない、またはbefore_sizeが0でないときにここに入る
	//before_sizeに関しては，物品がすべて取り出されるときの最後に取り出した物品を検知するための条件
	//別に分けても構わない
	if((msg->msgTMSInfo.empty() == false) || (ics_before_size != 0)){

		//物品が増えている場合
		//物品が増える場合は必ず末尾に1つだけ(ICS側のプログラムによる)
		if(ics_before_size < msg->msgTMSInfo.size()){

			//UPDATE process
			//※本当は書き込んであるかどうかを判定してADDかUPDATEかを判定しなくてはならない。
			//　役割的には微妙だが,DB_writer内でSQLで確認しても良い。役割を厳密にするならDB_readerを通じて取得する必要がある。
			//　今は実装していない。
            sprintf(query2, "UPDATE tmsdb.objects_present AS A SET A.rostime=%lu,A.x=%f,A.y=%f,A.z=%f,A.theta=%f,A.weight=%f,A.state=%d,A.place=%d WHERE id=%d;",
					msg->msgTMSInfo.back().tMeasuredTime.toNSec(), msg->msgTMSInfo.back().fX, msg->msgTMSInfo.back().fY, msg->msgTMSInfo.back().fZ,
					msg->msgTMSInfo.back().fTheta, msg->fWeight.back(), msg->msgTMSInfo.back().iState, msg->msgTMSInfo.back().iPlace, msg->msgTMSInfo.back().iID);
			//			printf("%s\n",query2);
			if (mysql_query(conn, query2)) {
				fprintf(stderr, "%s\n", mysql_error(conn));
				printf("Write error!\n");
			}

			//INSERT process
            sprintf(query1, "INSERT INTO tmsdb.objects_history(rostime,id,x,y,z,theta,weight,state,place) VALUES (%lu,%d,%f,%f,%f,%f,%f,%d,%d);",
					msg->msgTMSInfo.back().tMeasuredTime.toNSec(), msg->msgTMSInfo.back().iID, msg->msgTMSInfo.back().fX, msg->msgTMSInfo.back().fY, msg->msgTMSInfo.back().fZ,
					msg->msgTMSInfo.back().fTheta, msg->fWeight.back(), msg->msgTMSInfo.back().iState, msg->msgTMSInfo.back().iPlace);
			printf("%s\n",query1);
			if (mysql_query(conn, query1)) {
				fprintf(stderr, "%s\n", mysql_error(conn));
				printf("Write error!\n");
			}
		}

		//物品が減っている場合
		//この場合は配列中のIDが詰められるので、どの物品がなくなったかを判定する必要がある
		//前からforで見て、異なったIDになったところで分かる
		//<vector>.clear()は確保済の領域自体は消さずに中身のみclearするので,Segmentation Faultは生じない
		else if(ics_before_size > msg->msgTMSInfo.size()){
			for(unsigned int re_i=0;re_i<ics_before_size;re_i++){
				//1つ前の情報(1つの物品だけ少ない)と今回の情報で異なる場合

				//挿入されていた物品がすべて取り出されたときにここに入る
				//UPDATE、INSERTは最後に取り出されたもの(すなわちbefore_stateのfront)に対して行う
				//msgTMSInfoがemptyになる影響で取り出し時刻が分からなくなるため、msg直下のtMeasuredTimeを利用する。
				if(msg->msgTMSInfo.empty() == true){

					//UPDATE process
                    sprintf(query2, "UPDATE tmsdb.objects_present AS A SET A.rostime=%lu,A.weight=%f,A.state=%d,A.place=%d WHERE id=%d;",
							msg->tMeasuredTime.toNSec(), ics_before_state.front().weight, 0, 0, ics_before_state.front().iID);
					//					printf("%s\n",query2);
					if (mysql_query(conn, query2)) {
						fprintf(stderr, "%s\n", mysql_error(conn));
						printf("Write error!\n");
					}

					//INSERT process
                    sprintf(query1, "INSERT INTO tmsdb.objects_history(rostime,id,weight,state,place) VALUES (%lu,%d,%f,%d,%d);",
							msg->tMeasuredTime.toNSec(), ics_before_state.front().iID, ics_before_state.front().weight, 0, 0);
					printf("%s\n",query1);
					if (mysql_query(conn, query1)) {
						fprintf(stderr, "%s\n", mysql_error(conn));
						printf("Write error!\n");
					}
				}

				else if(ics_before_state[re_i].iID != msg->msgTMSInfo[re_i].iID){

					//UPDATE process
                    sprintf(query2, "UPDATE tmsdb.objects_present AS A SET A.rostime=%lu,A.weight=%f,A.state=%d,A.place=%d WHERE id=%d;",
							msg->msgTMSInfo[re_i].tMeasuredTime.toNSec(), msg->fWeight[re_i], 0, 0, ics_before_state[re_i].iID);
					//					printf("%s\n",query2);
					if (mysql_query(conn, query2)) {
						fprintf(stderr, "%s\n", mysql_error(conn));
						printf("Write error!\n");
					}

					//INSERT process
                    sprintf(query1, "INSERT INTO tmsdb.objects_history(rostime,id,weight,state,place) VALUES (%lu,%d,%f,%d,%d);",
							msg->msgTMSInfo.back().tMeasuredTime.toNSec(), ics_before_state[re_i].iID, ics_before_state[re_i].weight, 0, 0);
					printf("%s\n",query1);
					if (mysql_query(conn, query1)) {
						fprintf(stderr, "%s\n", mysql_error(conn));
						printf("Write error!\n");
					}

					break;
				}
			}
		}

		//その他の場合・・・既存物品のUPDATEのみ行う。
		else{
			for(unsigned int re_i=0;re_i<msg->msgTMSInfo.size();re_i++){
                sprintf(query2, "UPDATE tmsdb.objects_present AS A SET A.rostime=%lu,A.x=%f,A.y=%f,A.z=%f,A.theta=%f,A.weight=%f,A.state=%d,A.place=%d WHERE id=%d;",
						msg->msgTMSInfo[re_i].tMeasuredTime.toNSec(), msg->msgTMSInfo[re_i].fX, msg->msgTMSInfo[re_i].fY, msg->msgTMSInfo[re_i].fZ,
						msg->msgTMSInfo[re_i].fTheta, msg->fWeight[re_i], msg->msgTMSInfo[re_i].iState, msg->msgTMSInfo[re_i].iPlace, msg->msgTMSInfo[re_i].iID);
				//				printf("%s\n",query2);
				if (mysql_query(conn, query2)) {
					fprintf(stderr, "%s\n", mysql_error(conn));
					printf("Write error!\n");
				}
			}
		}
	}

	//before_ICSの更新
	ics_before_state.clear();
	for(unsigned int re_i=0;re_i<msg->msgTMSInfo.size();re_i++){
		temp_ics_before_state.iID = msg->msgTMSInfo[re_i].iID;
		temp_ics_before_state.iState = msg->msgTMSInfo[re_i].iState;
		temp_ics_before_state.x = msg->msgTMSInfo[re_i].fX;
		temp_ics_before_state.y = msg->msgTMSInfo[re_i].fY;
		temp_ics_before_state.weight = msg->fWeight[re_i];
		ics_before_state.push_back(temp_ics_before_state);
	}
	//beforeサイズの更新
	ics_before_size = msg->msgTMSInfo.size();

}

void DBwriteCallback_object_rev2(const tms_msg_ss::ics_object_data::ConstPtr& msg){

	//INSERT query
	char query1[1024];
	//UPDATE query
	char query2[1024];

	Ics_before_state temp_ics_before_state;

	//ROS_INFO("1");


	for(unsigned int i=0;i<msg->msgTMSInfo.size();i++){

		//UPDATE process
		//初回writer立ち上げ時は必ず書き換える
		//知的収納庫に物品が存在する場合は常に書き換える(state == 1)
		//存在しない場合は，存在しなくなった1回だけ書き換える(state == 1 かつ before_state == 0)
		if(ics_before_state_F_flag==true){		//初回は初期値として全てUPDATE，state == 1のときは知的収納庫に入っているので必ず書き換える.
			ROS_INFO("First Update");
			sprintf(query2, "UPDATE tmsdb.objects_present AS A SET A.rostime=%lu,A.x=%f,A.y=%f,A.z=%f,A.theta=%f,A.weight=%f,A.state=%d,A.place=%d WHERE id=%d;",
					msg->msgTMSInfo[i].tMeasuredTime.toNSec(), msg->msgTMSInfo[i].fX, msg->msgTMSInfo[i].fY, msg->msgTMSInfo[i].fZ,
					msg->msgTMSInfo[i].fTheta, msg->fWeight[i], msg->msgTMSInfo[i].iState, msg->msgTMSInfo[i].iPlace, msg->msgTMSInfo[i].iID);
			printf("%s\n",query2);
			if (mysql_query(conn, query2)) {
				fprintf(stderr, "%s\n", mysql_error(conn));
				printf("Write error!\n");
			}
		}

		else if(msg->msgTMSInfo[i].iState == 1){
			//ROS_INFO("istate==1");
			sprintf(query2, "UPDATE tmsdb.objects_present AS A SET A.rostime=%lu,A.x=%f,A.y=%f,A.z=%f,A.theta=%f,A.weight=%f,A.state=%d,A.place=%d WHERE id=%d;",
					msg->msgTMSInfo[i].tMeasuredTime.toNSec(), msg->msgTMSInfo[i].fX, msg->msgTMSInfo[i].fY, msg->msgTMSInfo[i].fZ,
					msg->msgTMSInfo[i].fTheta, msg->fWeight[i], msg->msgTMSInfo[i].iState, msg->msgTMSInfo[i].iPlace, msg->msgTMSInfo[i].iID);
			printf("%s\n",query2);
			if (mysql_query(conn, query2)) {
				fprintf(stderr, "%s\n", mysql_error(conn));
				printf("Write error!\n");
			}
		}
		else if((msg->msgTMSInfo[i].iState == 0) && (ics_before_state[i].iState == 1)){
			//ROS_INFO("istate = 0 & b_istate = 1");
			sprintf(query2, "UPDATE tmsdb.objects_present AS A SET A.rostime=%lu,A.x=%f,A.y=%f,A.z=%f,A.theta=%f,A.weight=%f,A.state=%d,A.place=%d WHERE id=%d;",
					msg->msgTMSInfo[i].tMeasuredTime.toNSec(), -10.0, -10.0, -10.0,
					-10.0, -10.0, msg->msgTMSInfo[i].iState, 0, msg->msgTMSInfo[i].iID);
			printf("%s\n",query2);
			if (mysql_query(conn, query2)) {
				fprintf(stderr, "%s\n", mysql_error(conn));
				printf("Write error!\n");
			}
		}

		//INSERT process
		//1つ前の状態と状態が変化した場合(挿入時/取り出し時)に更新
		//writer起動時にはINSERTを必ず行い，実状態に合わせる
		if((ics_before_state_F_flag==true) ||(msg->msgTMSInfo[i].iState != ics_before_state[i].iState)){
			if(msg->msgTMSInfo[i].iState == 1)	//iState == 1，すなわち物品が知的収納庫に存在するとき
			{
				sprintf(query1, "INSERT INTO tmsdb.objects_history(rostime,id,x,y,z,theta,weight,state,place) VALUES (%lu,%d,%f,%f,%f,%f,%f,%d,%d);",
						msg->msgTMSInfo[i].tMeasuredTime.toNSec(), msg->msgTMSInfo[i].iID, msg->msgTMSInfo[i].fX, msg->msgTMSInfo[i].fY, msg->msgTMSInfo[i].fZ,
						msg->msgTMSInfo[i].fTheta, msg->fWeight[i], msg->msgTMSInfo[i].iState, msg->msgTMSInfo[i].iPlace);
				printf("%s\n",query1);
				if (mysql_query(conn, query1)) {
					fprintf(stderr, "%s\n", mysql_error(conn));
					printf("Write error!\n");
				}
			}

			else	//iState == 0，すなわち物品が知的収納庫に存在しない時
			{
				if(ics_before_state_F_flag == true){
					sprintf(query1, "INSERT INTO tmsdb.objects_history(rostime,id,weight,state,place) VALUES (%lu,%d,%f,%d,%d);",
							msg->msgTMSInfo[i].tMeasuredTime.toNSec(), msg->msgTMSInfo[i].iID, 0.0, 0, 0);
					printf("%s\n",query1);
					if (mysql_query(conn, query1)) {
						fprintf(stderr, "%s\n", mysql_error(conn));
						printf("Write error!\n");
					}
				}
				else{
					sprintf(query1, "INSERT INTO tmsdb.objects_history(rostime,id,weight,state,place) VALUES (%lu,%d,%f,%d,%d);",
							msg->msgTMSInfo[i].tMeasuredTime.toNSec(), msg->msgTMSInfo[i].iID, ics_before_state[i].weight, 0, 0);
					printf("%s\n",query1);
					if (mysql_query(conn, query1)) {
						fprintf(stderr, "%s\n", mysql_error(conn));
						printf("Write error!\n");
					}
				}
			}
		}

	}

	//before_ICSの更新
	ics_before_state.clear();
	for(unsigned int re_i=0;re_i<msg->msgTMSInfo.size();re_i++){
		temp_ics_before_state.iID = msg->msgTMSInfo[re_i].iID;
		temp_ics_before_state.iState = msg->msgTMSInfo[re_i].iState;
		temp_ics_before_state.x = msg->msgTMSInfo[re_i].fX;
		temp_ics_before_state.y = msg->msgTMSInfo[re_i].fY;
		if(msg->msgTMSInfo[re_i].iState == 1){
			temp_ics_before_state.weight = msg->fWeight[re_i];
		}
		ics_before_state.push_back(temp_ics_before_state);
	}
	//beforeサイズの更新
//	ics_before_size = msg->msgTMSInfo.size();

	ics_before_state_F_flag = false;

}

//人の歩行軌跡の記録
void fss_person_trajectory_data(const tms_msg_ss::fss_person_trajectory_data::ConstPtr& msg){

	//query to MySQL
	char query1[1024];	//INSERT用クエリ
	char query2[1024];	//UPDATE用クエリ

//	ROS_INFO("DBwriteCallback_fss_person_trajectory_data : in\n");

	if(msg->msgTMSInfo.empty() == false){
		//UPDATE(人の現在位置を更新)
        sprintf(query2, "UPDATE tmsdb.person_present AS A SET A.rostime=%lu,A.x=%f,A.y=%f,A.z=%f,A.theta=%f,A.state=%d,A.place=%d WHERE id=%d;",
				msg->msgTMSInfo.back().tMeasuredTime.toNSec(), msg->msgTMSInfo.back().fX, msg->msgTMSInfo.back().fY, msg->msgTMSInfo.back().fZ,
				msg->msgTMSInfo.back().fTheta, msg->msgTMSInfo.back().iState, msg->msgTMSInfo.back().iPlace, msg->msgTMSInfo.back().iID);
//		printf("%s\n",query1);
		if (mysql_query(conn, query2)) {
			fprintf(stderr, "%s\n", mysql_error(conn));
			printf("Write error!(person_present)\n");
			printf("%s", query2);
		}
	}

	if(msg->msgTMSInfo.empty() == false){
		//配列が同じ数なら書き込まない(b_msgTMSInfo_personで保持)
		//異なっていれば、「差分だけ」SQL書き込み
		if(msg->msgTMSInfo.size() != b_msgTMSInfo_person.size()){
			for(unsigned int i=b_msgTMSInfo_person.size();i<msg->msgTMSInfo.size();i++){
				sprintf(query1,
                        "INSERT INTO tmsdb.person_history(rostime,id,x,y,z,theta,state,place) VALUES (%lu,%d,%f,%f,%f,%f,%d,%d)",
						msg->msgTMSInfo[i].tMeasuredTime.toNSec(), msg->msgTMSInfo[i].iID,
						msg->msgTMSInfo[i].fX, msg->msgTMSInfo[i].fY, msg->msgTMSInfo[i].fZ, msg->msgTMSInfo[i].fTheta,
						msg->msgTMSInfo[i].iState, msg->msgTMSInfo[i].iPlace);
//				printf("%s\n", query1);
				if (mysql_query(conn, query1)) {
					fprintf(stderr, "%s\n", mysql_error(conn));
					printf("Write error!(person_history)\n");
				}
			}
		}
		//前回のperson_history_dataを保存
		b_msgTMSInfo_person = msg->msgTMSInfo;
	}
}

void bas_behavior_data(const tms_msg_sa::bas_behavior_data::ConstPtr& msg){

	//query to MySQL
	char query1[1024];	//INSERT用
	char query2[1024];	//UPDATE用

	//空白データかどうか？
	if(msg->iBehaviorMerged.empty() == false){

		//入っている推定結果配列のすべての要素に対して以下の処理を行う
		for(unsigned int i=0;i<msg->iBehaviorMerged.size();i++){

			if(b_bas_behavior_data.iBehaviorMerged.empty() == false){
			//ROS_INFO("10");
				//新しい要素(1つ)に対してはインサート文の発行
				if(i == msg->iBehaviorMerged.size()-1){
					sprintf(query1,
                            "INSERT INTO tmsdb.person_behavior(rostime,id,behavior) VALUES (%lu,%d,%d);",
                            //"UPDATE tmsdb.person_history AS A SET A.state=%d WHERE A.rostime >= %lu AND A.rostime < %lu;",
							msg->tTimeStamp[i].toNSec(), 32, msg->iBehaviorMerged[i]);

					ROS_INFO("11");

					if (mysql_query(conn, query1)) {
						fprintf(stderr, "%s\n", mysql_error(conn));
						ROS_ERROR("Write error!(person_behavior_query1)\n");
						ROS_INFO("%s",query1);
					}

				}

				//一つ前の推定結果と今回の推定結果が同じでない要素についてUPDATE
				else if(msg->iBehaviorMerged[i] != b_bas_behavior_data.iBehaviorMerged[i]){
					ROS_INFO("12");

					sprintf(query2,
                            "UPDATE tmsdb.person_behavior AS A SET A.behavior=%d,A.id=31 WHERE A.rostime = %lu;",
                            //"UPDATE tmsdb.person_history AS A SET A.state=%d WHERE A.rostime >= %lu AND A.rostime < %lu;",
							msg->iBehaviorMerged[i], msg->tTimeStamp[i].toNSec());

					if (mysql_query(conn, query2)) {
						fprintf(stderr, "%s\n", mysql_error(conn));
						ROS_ERROR("Write error!(person_behavior_query2)\n");
						ROS_INFO("%s",query2);
					}
				}
				else{
					//ROS_INFO("not changed");
				}
			}
			else{
				ROS_INFO("FIRST BEHAVIOR");
			}
		}
		//		}
		//ROS_INFO("13");
		//b_bas_behavior_data.iBehaviorMerged = msg->iBehaviorMerged;
		b_bas_behavior_data = *msg;
	}
}


//PCDを更新(アップデート or 挿入)
bool tmsdb_file_conservation_func1(tms_msg_db::tmsdb_file_conservation::Request &req, tms_msg_db::tmsdb_file_conservation::Response &res){

	//query to MySQL
	char query1[1024];	//INSERT用クエリ
//	char query2[1024];	//UPDATE用クエリ

	std::string filename;
    char temp_filename[256];	sprintf(temp_filename, "/home/tmsdb/pcl_data/%lu_%d_robot%d.pcd", req.rostime,req.id,1);	//現状(面倒くさいので)smartpal4=1にidを固定
	filename = std::string(temp_filename);	std::cout << filename << std::endl;

	sprintf(query1,
            "INSERT INTO tmsdb.pcl_data(rostime,id,pcd_file,get_x,get_y,get_z,get_theta) VALUES (%lu,%d,%s,%f,%f,%f,%f);",
			req.rostime,req.id,req.filename.c_str(),req.get_x,req.get_y,req.get_z,req.get_theta);
	ROS_INFO("%s",query1);

	if (mysql_query(conn, query1)) {
		fprintf(stderr, "%s\n", mysql_error(conn));
		ROS_ERROR("Write error!(pcl_data_query1)\n");
		ROS_INFO("%s",query1);
		res.success = 1;
		res.message = "failed_INSERT_TO_PCL_DATA";
	}
	else{
		res.success = 0;
		res.message = "success_INSERT_TO_PCL_DATA";
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg (req.cloud, *cloud2);
	pcl::io::savePCDFileASCII (filename, *cloud2);

	return true;
}

bool tmsdb_objects_data_func1(tms_msg_db::tmsdb_objects_data::Request &req, tms_msg_db::tmsdb_objects_data::Response &res){

	//query to MySQL
	char query1[1024];	//INSERT用クエリ
//	char query2[1024];	//UPDATE用クエリ

	sprintf(query1,
            "INSERT INTO tmsdb.objects_history(rostime,id,x,y,z,theta,state,place) VALUES (%lu,%d,%f,%f,%f,%f,%d,%d);",
			req.rostime,req.id,req.x,req.y,req.z,req.theta,req.state,req.place);
	ROS_INFO("%s",query1);

	if (mysql_query(conn, query1)) {
		fprintf(stderr, "%s\n", mysql_error(conn));
		ROS_ERROR("Write error!(objects_data_query1)\n");
		ROS_INFO("%s",query1);
		res.success = 1;
		res.message = "failed_INSERT_TO_PCL_DATA";
	}
	else{
		res.success = 0;
		res.message = "success_INSERT_TO_OPJECT_DATA";
	}

	return true;
}

bool tmsdb_ods_object_data_func1(tms_msg_db::tmsdb_ods_object_data::Request &req, tms_msg_db::tmsdb_ods_object_data::Response &res){

	//query to MySQL
	char query1[1024];	//INSERT用クエリ
//	char query2[1024];	//UPDATE用クエリ

	for(unsigned int i=0;i<req.srvTMSInfo.size();i++){

		//UPDATE process
		sprintf(query1, "UPDATE tmsdb.objects_present AS A SET A.rostime=%lu,A.x=%f,A.y=%f,A.z=%f,A.theta=%f,A.weight=%f,A.state=%d,A.place=%d WHERE id=%d;",
				req.srvTMSInfo[i].tMeasuredTime.toNSec(), req.srvTMSInfo[i].fX, req.srvTMSInfo[i].fY, req.srvTMSInfo[i].fZ,
				req.srvTMSInfo[i].fTheta, -1.0, req.srvTMSInfo[i].iState, req.srvTMSInfo[i].iPlace, req.srvTMSInfo[i].iID);
		printf("%s\n",query1);
		if (mysql_query(conn, query1)) {
			fprintf(stderr, "%s\n", mysql_error(conn));
			printf("Write error!\n");
		}
	}

	return true;
}

void DBwriteCallback_robot_current_data(const tms_msg_rc::robot_current_data::ConstPtr& msg){
    char query1[1024];


    //UPDATE process
    if(msg->iID == 2){ //smartpal5
        sprintf(query1, "UPDATE robots_present AS A SET A.rostime=%lu,A.x=%f,A.y=%f,A.z=%f,A.theta=%f,A.joint_waist_low=%f,A.joint_waist_high=%f,A.joint_arm_right_1=%f,A.joint_arm_right_2=%f,A.joint_arm_right_3=%f,A.joint_arm_right_4=%f, A.joint_arm_right_5=%f,A.joint_arm_right_6=%f,A.joint_arm_right_7=%f,A.joint_arm_right_g=%f,A.joint_arm_left_1=%f,A.joint_arm_left_2=%f,A.joint_arm_left_3=%f,A.joint_arm_left_4=%f,A.joint_arm_left_5=%f,A.joint_arm_left_6=%f,A.joint_arm_left_7=%f,A.joint_arm_left_g=%f,A.state=%d,A.place=%d WHERE id=%d;",
                        msg->tMeasuredTime.toNSec(),
                        msg->fX,
                        msg->fY,
                        msg->fZ,
                        msg->fTheta,
                        msg->fJonit_waist_low,
                        msg->fJonit_waist_high,
                        msg->fJonit_arm_right_1,
                        msg->fJonit_arm_right_2,
                        msg->fJonit_arm_right_3,
                        msg->fJonit_arm_right_4,
                        msg->fJonit_arm_right_5,
                        msg->fJonit_arm_right_6,
                        msg->fJonit_arm_right_7,
                        msg->fJonit_arm_right_g,
                        msg->fJonit_arm_left_1,
                        msg->fJonit_arm_left_2,
                        msg->fJonit_arm_left_3,
                        msg->fJonit_arm_left_4,
                        msg->fJonit_arm_left_5,
                        msg->fJonit_arm_left_6,
                        msg->fJonit_arm_left_7,
                        msg->fJonit_arm_left_g,
                        msg->iState,
                        msg->iPlace,
                        msg->iID
                );

        if (mysql_query(conn, query1)) {
            fprintf(stderr, "%s\n", mysql_error(conn));
            printf("Write error! (robot_current_data)\n");
        }
    }
    else{
    ROS_INFO("msg TYPE is wrong : UPDATE PROCESS");
    }

}

//main関数
int main(int argc, char **argv){

	if ((fp = fopen("test.log", "w")) == NULL) {
		printf("file open error!!\n");
		exit(EXIT_FAILURE);
	}

	int count = 0;

	ros::init(argc, argv, "db_writer");
	ros::NodeHandle n;
	ros::Rate r(10);
	printf("db_writer : init\n");

	// MySQL connection
	conn = mysql_init(NULL);
	if (!mysql_real_connect(conn, DBHOST, DBUSER, DBPASS, DBNAME, 3306, NULL, 0)) {
	fprintf(stderr, "%s\n", mysql_error(conn));
	exit(1);
	}
	printf("MySQL(tmsdb) opened.\n");

	//初期化
	for(count=0; count<MAX_OBJECT_ICS;count++){
		bchecker_ics[count].x_b = -9999;
		bchecker_ics[count].y_b = -9999;
		bchecker_ics[count].z_b = -9999;
	}

	for(unsigned int i=0;i<ics_before_state.size();i++){
		ics_before_state[i].iState = 0;
	}

	ros::Subscriber sub0 = n.subscribe("fss_smartpal_data", 100, DBwriteCallback);
	ros::Subscriber sub1 = n.subscribe("fss_roomba_data", 100, DBwriteCallback);
	ros::Subscriber sub2 = n.subscribe("fss_chair_data", 100, DBwriteCallback);
	ros::Subscriber sub3 = n.subscribe("fss_wagon_data", 100, DBwriteCallback);

	ros::Subscriber sub4 = n.subscribe("ics_object_data", 100, DBwriteCallback_object_rev2);

	ros::Subscriber sub5 = n.subscribe("fss_person_trajectory_data", 100, fss_person_trajectory_data);
	ros::Subscriber sub6 = n.subscribe("bas_behavior_data", 100, bas_behavior_data);
    ros::Subscriber sub7 = n.subscribe("robot_current_data", 100, DBwriteCallback_robot_current_data);

    ros::Subscriber sub8 = n.subscribe("kkp_current_data",100,DBwriteCallback);

//	ros::ServiceServer service1 = n.advertiseService("tmssrv_modify_missing_objects", tmssrv_modify_missing_objects_func);
//	ros::ServiceServer service2 = n.advertiseService("modify_person_behavior", modify_person_behavior);
	ros::ServiceServer service01 = n.advertiseService("tmsdb_file_conservation", tmsdb_file_conservation_func1);
	ros::ServiceServer service02 = n.advertiseService("tmsdb_objects_data_1", tmsdb_objects_data_func1);
	ros::ServiceServer service03 = n.advertiseService("tmsdb_ods_object_data", tmsdb_ods_object_data_func1);

	ros::spin();

	// close connection
	mysql_close(conn);

	return 0;
}
