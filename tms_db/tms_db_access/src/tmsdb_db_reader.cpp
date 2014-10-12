//include for ROS
#include "ros/ros.h"
#include <unistd.h>
#include <boost/date_time/local_time/local_time.hpp> 
#include "tms_msg_db/tmsdb_planning.h"
#include "tms_msg_db/tmsdb_obj_dt.h"
#include "tms_msg_db/tmsdb_missing_objects_checker.h"
#include "tms_msg_db/tmsdb_data.h"
#include "tms_msg_db/tmsdb_robot_trajectory_data.h"

#include "tms_msg_db/tmsdb_get_robot_info.h"
#include "tms_msg_db/tmsdb_get_objects_info.h"
#include "tms_msg_db/tmsdb_get_person_info.h"
#include "tms_msg_db/tmsdb_get_person_behavior_info.h"
#include "tms_msg_db/tmsdb_get_furnitures_info.h"
#include "tms_msg_db/tmsdb_get_movable_furnitures_info.h"
#include "tms_msg_db/tmsdb_get_robots_info.h"
#include "tms_msg_db/tmsdb_get_pcd_info.h"
#include "tms_msg_db/tmsdb_get_task_list.h"
#include "tms_msg_db/tmsdb_get_unknown_object.h"
#include "tms_msg_rp/tms_rs_get_object_info_ex.h"
//#include "tms_msg_rp/tms_rs_get_around_object.h"
//#include "tms_msg_rp/tms_rs_get_object_info_rt.h"

//include for MySQL
#include <stdio.h>
#include <stdlib.h>
#include <mysql/mysql.h>

//include for PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

//MySQL login Macro
#define DBHOST "192.168.4.170"
#define DBUSER "root"
#define DBPASS "tmsdb"
#define DBNAME "tmsdb"

//#define DBHOST "133.5.20.38"
//#define DBUSER "user13"
//#define DBPASS "BF2Ax7H4"
//#define DBNAME "tmsdb"

struct Id_name_itype{
	int id;
	char name[256];
	int type;
};

//起動時に1回id,name,itypeの対応付けをDBから取り出す
//とりあえずmain関数内で処理
Id_name_itype id_name_itype[1024];

MYSQL *conn;	//MySQL Connector
MYSQL_RES *result;	//MySQL result (fields)
MYSQL_ROW row;	//MySQL field

FILE *fp;

//objects_presentより,入力で指定されたフィールドを持つタップルを取得する
//現状はstate=0のものを取得する
bool tmsdb_get_objects_info_func1(tms_msg_db::tmsdb_get_objects_info::Request &req, tms_msg_db::tmsdb_get_objects_info::Response &res){

	char query[1024];
	MYSQL_ROW tmp_row;
	unsigned int i=0;

	char *endptr;

	sprintf(query, "SELECT rostime,id,x,y,z,weight,state,place FROM tmsdb.objects_present WHERE state=0");
	printf("%s\n",query);
	mysql_query(conn, query);
	result = mysql_use_result(conn);

	//output table name
	printf("MySQL Tables in mysql database1:\n");
//	if(true){
//		ROS_INFO("1");
//		if(mysql_fetch_row(result) == NULL){
//			ROS_INFO("NULL");
//		}
//		exit(0);
//	}
	//ROS_INFO("%d",(int)mysql_num_rows(result));

	//exist more than 2_rows,fetch the rows
	while ((row = mysql_fetch_row(result)) != NULL){

		tmp_row = row;
		res.rostime.push_back(strtoll(tmp_row[0],&endptr, 10));
		res.id.push_back(atoi(tmp_row[1]));
		res.x.push_back((float)atof(tmp_row[2]));
		res.y.push_back((float)atof(tmp_row[3]));
		res.z.push_back((float)atof(tmp_row[4]));
		res.weight.push_back((float)atof(tmp_row[5]));
		res.state.push_back(atoi(tmp_row[6]));
		res.place.push_back(atof(tmp_row[7]));
		printf("%lu, %d, %f, %f, %f, %f, %d, %d\n", res.rostime[i], res.id[i], res.x[i], res.y[i], res.z[i], res.weight[i], res.state[i], res.place[i]);
		i++;
	}

	if(mysql_num_rows(result) != 0)
		mysql_free_result(result);

	return true;
}

bool tmsdb_get_objects_info_func2(tms_msg_db::tmsdb_get_objects_info::Request &req, tms_msg_db::tmsdb_get_objects_info::Response &res){

	char query[1024];
	MYSQL_ROW tmp_row;
	unsigned int j=0;

	char *endptr;

	//historyより，reqで指定したidの最新の物品状態を取得する
	for(unsigned int i=0;i<req.id.size();i++){
		sprintf(query, "SELECT rostime,id,x,y,z,weight,state,place FROM tmsdb.objects_history WHERE rostime=(SELECT MAX(rostime) FROM tmsdb.objects_history WHERE id=%d)"
				, req.id[i]);
		printf("%s\n",query);
		mysql_query(conn, query);
		result = mysql_use_result(conn);

		//output table name
		//printf("MySQL Tables in mysql database1:\n");

		//exist more than 2_rows,fetch the rows
		while ((row = mysql_fetch_row(result)) != NULL){
//			printf("1\n");
			tmp_row = row;
			printf("%s,%s,%s,%s,%s,%s,%s,%s\n",tmp_row[0],tmp_row[1],tmp_row[2],tmp_row[3],tmp_row[4],tmp_row[5],tmp_row[6],tmp_row[7]);
			res.rostime.push_back(strtoll(tmp_row[0],&endptr, 10));
			res.id.push_back(atoi(tmp_row[1]));
			res.weight.push_back((float)atof(tmp_row[5]));
			res.state.push_back(atoi(tmp_row[6]));
			res.place.push_back(atof(tmp_row[7]));
//			if(state!=0){
//			res.x.push_back((float)atof(tmp_row[2]));
//			res.y.push_back((float)atof(tmp_row[3]));
//			res.z.push_back((float)atof(tmp_row[4]));
//			}
//			printf("%lu, %d, %f, %f, %f, %f, %d, %d\n", res.rostime[j], res.id[j], res.x[j], res.y[j], res.z[j], res.weight[j], res.state[j], res.place[j]);
			j++;
		}
	}

	if(mysql_num_rows(result) != 0)
		mysql_free_result(result);

	return true;
}

// by pyo
bool tmsdb_get_current_objects_info(tms_msg_db::tmsdb_get_objects_info::Request &req, tms_msg_db::tmsdb_get_objects_info::Response &res){

	char query[1024];
	MYSQL_ROW tmp_row;
	unsigned int i=0;

	char *endptr;

	sprintf(query, "SELECT rostime,id,x,y,z,theta,weight,state,place FROM tmsdb.objects_present");
	printf("%s\n",query);
	mysql_query(conn, query);
	result = mysql_use_result(conn);

	printf("MySQL Tables in mysql database:\n");

	while ((row = mysql_fetch_row(result)) != NULL){
		tmp_row = row;
		res.rostime.push_back(strtoll(tmp_row[0],&endptr, 10));
		res.id.push_back(atoi(tmp_row[1]));
		res.x.push_back((float)atof(tmp_row[2]));
		res.y.push_back((float)atof(tmp_row[3]));
		res.z.push_back((float)atof(tmp_row[4]));
		res.ry.push_back((float)atof(tmp_row[5]));
		res.weight.push_back((float)atof(tmp_row[6]));
		res.state.push_back(atoi(tmp_row[7]));
		res.place.push_back(atof(tmp_row[8]));
		printf("%lu, %d, %f, %f, %f, %f, %f, %d, %d\n", res.rostime[i], res.id[i], res.x[i], res.y[i], res.z[i], res.ry[i], res.weight[i], res.state[i], res.place[i]);
		i++;
	}

	if(mysql_num_rows(result) != 0)
		mysql_free_result(result);

	return true;
}

// by pyo
bool tmsdb_get_current_person_info(tms_msg_db::tmsdb_get_person_info::Request &req, tms_msg_db::tmsdb_get_person_info::Response &res)
{
	char query[1024];
	MYSQL_ROW tmp_row;
	unsigned int i=0;

	char *endptr;

	sprintf(query, "SELECT rostime,id,x,y,z,theta,state,place FROM tmsdb.person_present");
	printf("%s\n",query);
	mysql_query(conn, query);
	result = mysql_use_result(conn);

	printf("MySQL Tables in mysql database:\n");

	while ((row = mysql_fetch_row(result)) != NULL){
		tmp_row = row;
		res.rostime.push_back(strtoll(tmp_row[0],&endptr, 10));
		res.id.push_back(atoi(tmp_row[1]));
		res.x.push_back((float)atof(tmp_row[2]));
		res.y.push_back((float)atof(tmp_row[3]));
		res.z.push_back((float)atof(tmp_row[4]));
		res.theta.push_back((float)atof(tmp_row[5]));
		res.state.push_back(atoi(tmp_row[6]));
		res.place.push_back(atof(tmp_row[7]));
		printf("%lu, %d, %f, %f, %f, %f, %d, %d\n", res.rostime[i], res.id[i], res.x[i], res.y[i], res.z[i], res.theta[i], res.state[i], res.place[i]);
		i++;
	}

	if(mysql_num_rows(result) != 0)
		mysql_free_result(result);

	return true;
}

// by pyo
bool tmsdb_get_current_robot_info(tms_msg_db::tmsdb_get_robot_info::Request &req, tms_msg_db::tmsdb_get_robot_info::Response &res)
{
  char query[1024];
  MYSQL_ROW tmp_row;
  char *endptr;

  sprintf(query, "SELECT rostime,id,x,y,z,theta,joint_waist_low,joint_waist_high,joint_arm_right_1,joint_arm_right_2,joint_arm_right_3,joint_arm_right_4,joint_arm_right_5,joint_arm_right_6,joint_arm_right_7,joint_arm_right_g,joint_arm_left_1,joint_arm_left_2,joint_arm_left_3,joint_arm_left_4,joint_arm_left_5,joint_arm_left_6,joint_arm_left_7,joint_arm_left_g,state,place FROM tmsdb.robots_present");
  printf("%s\n",query);
  mysql_query(conn, query);
  result = mysql_use_result(conn);

  while ((row = mysql_fetch_row(result)) != NULL) {
    tmp_row = row;

    res.rostime.push_back(strtoll(tmp_row[0],&endptr, 10));
    res.id.push_back(atoi(tmp_row[1]));
    res.x.push_back((float)atof(tmp_row[2]));
    res.y.push_back((float)atof(tmp_row[3]));
    res.z.push_back((float)atof(tmp_row[4]));
    res.theta.push_back((float)atof(tmp_row[5]));
    res.joint_waist_low.push_back((float)atof(tmp_row[6]));
    res.joint_waist_high.push_back((float)atof(tmp_row[7]));
    res.joint_arm_right_1.push_back((float)atof(tmp_row[8]));
    res.joint_arm_right_2.push_back((float)atof(tmp_row[9]));
    res.joint_arm_right_3.push_back((float)atof(tmp_row[10]));
    res.joint_arm_right_4.push_back((float)atof(tmp_row[11]));
    res.joint_arm_right_5.push_back((float)atof(tmp_row[12]));
    res.joint_arm_right_6.push_back((float)atof(tmp_row[13]));
    res.joint_arm_right_7.push_back((float)atof(tmp_row[14]));
    res.joint_arm_right_g.push_back((float)atof(tmp_row[15]));
    res.joint_arm_left_1.push_back((float)atof(tmp_row[16]));
    res.joint_arm_left_2.push_back((float)atof(tmp_row[17]));
    res.joint_arm_left_3.push_back((float)atof(tmp_row[18]));
    res.joint_arm_left_4.push_back((float)atof(tmp_row[19]));
    res.joint_arm_left_5.push_back((float)atof(tmp_row[20]));
    res.joint_arm_left_6.push_back((float)atof(tmp_row[21]));
    res.joint_arm_left_7.push_back((float)atof(tmp_row[22]));
    res.joint_arm_left_g.push_back((float)atof(tmp_row[23]));
    res.state.push_back(atoi(tmp_row[24]));
    res.place.push_back(atoi(tmp_row[25]));
  }

  if(mysql_num_rows(result) != 0)
    mysql_free_result(result);

  return true;
}

bool tmsdb_get_furnitures_info_func1(tms_msg_db::tmsdb_get_furnitures_info::Request &req, tms_msg_db::tmsdb_get_furnitures_info::Response &res){

	char query[1024];
	MYSQL_ROW tmp_row;

	sprintf(query, "SELECT x,y,z FROM tmsdb.furnitures WHERE id=%d", req.furnitures_id);
	printf("%s", query);
	mysql_query(conn, query);
	result = mysql_use_result(conn);
	row = mysql_fetch_row(result);

	tmp_row = row;
	printf("%s,%s,%s\n",tmp_row[0],tmp_row[1],tmp_row[2]);
	res.furniture_x = (float)atof(tmp_row[0]);
	res.furniture_y = (float)atof(tmp_row[1]);
	res.furniture_z = (float)atof(tmp_row[2]);

	mysql_free_result(result);

	return true;
}

bool tmsdb_get_movable_furnitures_info_func1(tms_msg_db::tmsdb_get_movable_furnitures_info::Request &req, tms_msg_db::tmsdb_get_movable_furnitures_info::Response &res){

	char query[1024];
	MYSQL_ROW tmp_row;

	sprintf(query, "SELECT mfp.x,mfp.y,mfp.z,mfp.theta,g.width,g.depth,g.height,mfp.state,mfp.place FROM tmsdb.movable_furnitures_present AS mfp LEFT JOIN tmsdb.geometry AS g ON mfp.id=g.id WHERE mfp.id=%d",
			req.furnitures_id);
	printf("%s", query);
	mysql_query(conn, query);
	result = mysql_use_result(conn);
	row = mysql_fetch_row(result);

	tmp_row = row;
	printf("%s,%s,%s,%s,%s,%s,%s,%s,%s\n",tmp_row[0],tmp_row[1],tmp_row[2],tmp_row[3],tmp_row[4],tmp_row[5],tmp_row[6],tmp_row[7],tmp_row[8]);
	res.furniture_x = (float)atof(tmp_row[0]);
	res.furniture_y = (float)atof(tmp_row[1]);
	res.furniture_z = (float)atof(tmp_row[2]);
	res.furnitures_theta = (float)atof(tmp_row[3]);
	res.furnitures_width = (float)atof(tmp_row[4]);
	res.furnitures_depth = (float)atof(tmp_row[5]);
	res.furnitures_height = (float)atof(tmp_row[6]);
	res.furnitures_state = atoi(tmp_row[7]);
	res.furnitures_place = atoi(tmp_row[8]);

	mysql_free_result(result);

	return true;
}

bool tmsdb_get_robots_info_func1(tms_msg_db::tmsdb_get_robots_info::Request &req, tms_msg_db::tmsdb_get_robots_info::Response &res){

	char query[1024];
	MYSQL_ROW tmp_row;
	char *endptr;

	sprintf(query, "SELECT rostime,x,y,z,theta,state,place FROM tmsdb.robots_present WHERE id=%d", req.robots_id);
	printf("%s", query);
	mysql_query(conn, query);
	result = mysql_use_result(conn);


	while ((row = mysql_fetch_row(result)) != NULL){

		tmp_row = row;
		printf("%s,%s,%s,%s,%s,%s,%s\n",tmp_row[0],tmp_row[1],tmp_row[2],tmp_row[3],tmp_row[4],tmp_row[5],tmp_row[6]);

		res.rostime = strtoll(tmp_row[0],&endptr, 10);
		res.robots_x = (float)atof(tmp_row[1]);
		res.robots_y = (float)atof(tmp_row[2]);
		res.robots_z = (float)atof(tmp_row[3]);
		res.robots_theta = (float)atof(tmp_row[4]);
		res.robots_state = atoi(tmp_row[5]);
		res.robots_place = atoi(tmp_row[6]);

	}
	if(mysql_num_rows(result) != 0)
		mysql_free_result(result);

	return true;
}

bool tmsdb_get_person_info_func1(tms_msg_db::tmsdb_get_person_info::Request &req, tms_msg_db::tmsdb_get_person_info::Response &res){

/*
 * 分析のための人の行動履歴を2段階で抜き出す
 * (1)
 * 物品紛失時刻req.rostime[0]～それ以降でもっとも短い退出時までのperson_behaviorを取得する
 * すなわち、人が知的収納庫から物品を取り出し、退出するまでの人のbehaviorを抜き出す
 * (2)
 * 物品紛失時刻req.rostime[0]～(1)で抜き出した退出時刻までのperson_historyを取得する
 * すなわち、人が知的収納庫から物品を取り出し、退出するまでの人の移動履歴を抜き出す
 *
 * ※person_history＋person_behaviorの自然結合命令で取得したテーブルから一気に人の移動履歴を抜き出してもよいが
 * 　SQL命令が非常に複雑になるので2つに分けています。
 *   (1)で欲しい情報は退出(behavior=0)の時刻のみです。

 */


	char query1[1024];		//for person_behavior table
	MYSQL_ROW tmp_row_tmp;

	std::vector<long unsigned int> tmp_rostime;
	std::vector<unsigned int> tmp_id;
	std::vector<unsigned int> tmp_behavior;


	char query2[1024];		//for person_history table <- person_behavior table
	MYSQL_ROW tmp_row;
//	unsigned int j=0;

	char *endptr_tmp;
	char *endptr;

	//req.rostime[0]は物品紛失時の時刻
	//person_behaviorより，「req.rostime[0]以降」「それにもっとも近い、状態=退出になるまで」の全情報を抜き出す

	//	for(unsigned int i=0;i<req.id.size();i++){

		//req.rostime[0]は物品紛失時の時刻
		//person_behaviorより, 「req.rostime[0]以降」「それにもっとも近い、状態=退出になるまで」の全情報を抜き出す
		sprintf(query1,
				"SELECT * FROM tmsdb.person_behavior WHERE rostime>= %lu AND rostime<(SELECT MIN(rostime) FROM tmsdb.person_behavior WHERE rostime>= %lu AND behavior=0 ORDER BY rostime) ORDER BY rostime;",
				req.rostime[0], req.rostime[0]);
		ROS_INFO("%s",query1);
		mysql_query(conn, query1);
		result = mysql_use_result(conn);

		//ここにresult(SQL文の結果が入っているMYSQL_RES構造体)が空の場合
		//すなわち、上のSQL文の結果が「なし」の場合はコールバックを抜けるようにしないと該当時にSegmentationFaultになる

//		ROS_INFO("%d", mysql_num_rows(result));

		while ((row = mysql_fetch_row(result)) != NULL){
			tmp_row_tmp = row;
			printf("%s,%s,%s\n",tmp_row_tmp[0],tmp_row_tmp[1],tmp_row_tmp[2]);
			tmp_rostime.push_back(strtoll(tmp_row_tmp[0], &endptr_tmp, 10));
			tmp_id.push_back(atoi(tmp_row_tmp[1]));
			tmp_behavior.push_back(atoi(tmp_row_tmp[2]));
		}
		//result領域開放
		mysql_free_result(result);

		//person_historyより, 「物品紛失時以降」「状態=退出の時刻まで」の全情報を抜き出す
		sprintf(query2,
				"SELECT * FROM tmsdb.person_history WHERE rostime>=%lu AND rostime<%lu ORDER BY rostime;",
				req.rostime[0],tmp_rostime.back());		//tmp_rostime.back()は退出時刻
		ROS_INFO("%s",query2);
		mysql_query(conn, query2);
		result = mysql_use_result(conn);

		//output table name
		printf("MySQL: tmsdb_get_person_info_func1:\n");
		printf("query:%s\n", query2);

		//exist more than 2_rows,fetch the rows
		while ((row = mysql_fetch_row(result)) != NULL){
//			printf("1\n");
			tmp_row = row;
			//rostime, id, x, y, z, theta, state, place
			printf("%s,%s,%s,%s,%s,%s,%s,%s\n",tmp_row[0],tmp_row[1],tmp_row[2],tmp_row[3],tmp_row[4],tmp_row[5],tmp_row[6],tmp_row[7]);
			res.rostime.push_back(strtoll(tmp_row[0],&endptr, 10));
			res.id.push_back(atoi(tmp_row[1]));
			res.x.push_back((float)atof(tmp_row[2]));
			res.y.push_back((float)atof(tmp_row[3]));
			res.z.push_back((float)atof(tmp_row[4]));
			res.theta.push_back((float)atof(tmp_row[5]));
			res.state.push_back(atoi(tmp_row[6]));
			res.place.push_back(atof(tmp_row[7]));
//			if(state!=0){

//			}
//			printf("%lu, %d, %f, %f, %f, %f, %d, %d\n", res.rostime[j], res.id[j], res.x[j], res.y[j], res.z[j], res.weight[j], res.state[j], res.place[j]);
//			j++;
		}
//	}

	//result領域開放
	//ここでメモリの2重開放エラーを生じる場合があるため、『mysql_fetch_rowで情報を全て取得したのち使える関数』のmysql_num_rowsを用いる。
	//つまり、SELECT文の該当行が存在しなかった場合に開放しないようにするということ
	if(mysql_num_rows(result) != 0)
		mysql_free_result(result);

	return true;
}

bool tmsdb_get_person_info_func2(tms_msg_db::tmsdb_get_person_info::Request &req, tms_msg_db::tmsdb_get_person_info::Response &res){

	//指定した時刻以降の人の歩行軌跡を取得する

	char query1[1024];		//for person_behavior table
	MYSQL_ROW tmp_row;

	char *endptr_tmp;
	char *endptr;

	sprintf(query1, "SELECT rostime,id,x,y FROM tmsdb.person_history WHERE rostime>= %lu", req.rostime.front());
	ROS_INFO("%s",query1);
	mysql_query(conn, query1);
	result = mysql_use_result(conn);

	//ここにresult(SQL文の結果が入っているMYSQL_RES構造体)が空の場合
	//すなわち、上のSQL文の結果が「なし」の場合はコールバックを抜けるようにしないと該当時に~fetch_row(NULL)となり、SegmentationFaultになる
	if(result != NULL){
		while ((row = mysql_fetch_row(result)) != NULL){
			tmp_row = row;
			ROS_INFO("%s,%s,%s,%s",tmp_row[0],tmp_row[1],tmp_row[2],tmp_row[3]);
			res.rostime.push_back(strtoll(tmp_row[0],&endptr, 10));
			res.id.push_back(atoi(tmp_row[1]));
			res.x.push_back((float)atof(tmp_row[2]));
			res.y.push_back((float)atof(tmp_row[3]));
		}
		//result領域開放
		if(mysql_num_rows(result) != 0)
			mysql_free_result(result);
	}
	else{
		ROS_INFO("result = NULL");
	}

	printf("\n");

	return true;
}

//PCDファイルの取得：紐付けは家具idで行う
bool tmsdb_get_pcd_info_func1(tms_msg_db::tmsdb_get_pcd_info::Request &req, tms_msg_db::tmsdb_get_pcd_info::Response &res){

	char query[1024];
	MYSQL_ROW tmp_row;
	char *endptr;

	//本来は最新1つ分を持ち出すためMAX(rostime)の記述が必要
	//今はrostime=1のものを使用するので，トップの情報を利用することでエラーを吐いていないだけ
	sprintf(query, "SELECT * FROM tmsdb.pcl_data WHERE id=%d AND rostime=(SELECT MAX(rostime) FROM tmsdb.pcl_data);", req.id);
	printf("%s\n",query);
	if (mysql_query(conn, query)) {
		fprintf(stderr, "%s\n", mysql_error(conn));
		printf("Write error!(person_behavior_query)\n");
		printf("%s\n",query);
	}
	result = mysql_use_result(conn);

	//一意に引き出してくる情報が決まるならこれでよい
	//ない場合はここでエラーを吐くので注意
	row = mysql_fetch_row(result);

	tmp_row = row;
	printf("%s,%s,%s,%s,%s,%s,%s\n",tmp_row[0],tmp_row[1],tmp_row[2],tmp_row[3],tmp_row[4],tmp_row[5],tmp_row[6]);

	res.rostime = strtoll(tmp_row[0], &endptr, 10);
	res.pcd_file = tmp_row[2];
	res.get_x = (float)atof(tmp_row[3]);
	res.get_y = (float)atof(tmp_row[4]);
	res.get_z = (float)atof(tmp_row[5]);
	res.get_theta = atoi(tmp_row[6]);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	sensor_msgs::PointCloud2::Ptr client (new sensor_msgs::PointCloud2 ());
	pcl::io::loadPCDFile (tmp_row[2], *cloud);

	pcl::toROSMsg (*cloud, *client);
	res.cloud = *client;

	mysql_free_result(result);

	return true;
}

bool tmsdb_get_person_info_func3(tms_msg_db::tmsdb_get_person_behavior_info::Request &req, tms_msg_db::tmsdb_get_person_behavior_info::Response &res){

	char query[1024];
	MYSQL_ROW tmp_row;

	sprintf(query, "SELECT * FROM tmsdb.person_behavior WHERE rostime=(SELECT MAX(rostime) FROM tmsdb.person_behavior)", req.rostime);
	printf("%s", query);
	mysql_query(conn, query);
	result = mysql_use_result(conn);
	row = mysql_fetch_row(result);

	tmp_row = row;
	printf("%s,%s,%s\n",tmp_row[0],tmp_row[1],tmp_row[2]);
//	res.rostime = atoi(tmp_row[0]);
	res.id = atoi(tmp_row[1]);
	res.behavior = atoi(tmp_row[2]);

	mysql_free_result(result);
	return true;
}

bool tmsdb_robot_trajectory_data(tms_msg_db::tmsdb_robot_trajectory_data::Request &req, tms_msg_db::tmsdb_robot_trajectory_data::Response &res){

	char query[1024];
	char *endptr;
	int count = 0;
	ros::Time temp_time;
	tms_msg_db::tmsdb_data temp_tms_data;
//	uint64_t temp_nsec;

	ros::Time tMeasuredTime_s = req.tStartTime;
	ros::Time tMeasuredTime_e = req.tEndTime;
	uint64_t nsec_s = tMeasuredTime_s.toNSec();
	uint64_t nsec_e = tMeasuredTime_e.toNSec();

	//typeの取得　これがないとどのテーブルにアクセスしてよいか分からないため
	//これは、read_robots_history内ではなくmain関数内で1回か、
	//タイマーを利用して大域配列変数に入れる方がよいかもしれない
//	sprintf(query, "SELECT id_name.type FROM id_name WHERE id=%d;", msg->...);
//	result = mysql_use_result(conn);
//	while((row = mysql_fetch_row(result))){
//		itype = atoi(row[0]);
//	}

	//iType = 1 -> robots_historyから読み出し
	if(id_name_itype[req.iID].type == 1){
		sprintf(query, "SELECT rostime,id,x,y,z,theta,state,place FROM tmsdb.robots_history WHERE id=%d AND rostime>=%lu AND rostime<= %lu;",
				req.iID, nsec_s, nsec_e);
		printf("%s\n", query);

		mysql_query(conn, query);
		result = mysql_use_result(conn);
		while((row = mysql_fetch_row(result))){
//			ros::Time T_temp;
//			temp_tms_data.tMeasuredTime = T_temp.fromNSec(strtoll(row[0], &endptr, 10));
			temp_tms_data.tMeasuredTime.fromNSec(strtoll(row[0], &endptr, 10));
			temp_tms_data.iID = atoi(row[1]);
			temp_tms_data.iType = 1;
			temp_tms_data.fX = (float)(atof(row[2]));
			temp_tms_data.fY = (float)(atof(row[3]));
			temp_tms_data.fZ = (float)(atof(row[4]));
			temp_tms_data.fTheta = (float)(atof(row[5]));
			temp_tms_data.iPlace = atoi(row[7]);
			temp_tms_data.iState = atoi(row[6]);

			//responseへpush_back
			res.msgTMSInfo.push_back(temp_tms_data);
			count++;
		}
	}

	else if(id_name_itype[req.iID].type == 3){
		sprintf(query, "SELECT rostime,id,x,y,z,theta,state,place FROM tmsdb.robots_history WHERE id=%d AND rostime>=%lu AND rostime<= %lu;",
				req.iID, nsec_s, nsec_e);
		printf("%s\n", query);

		mysql_query(conn, query);
		result = mysql_use_result(conn);
		while((row = mysql_fetch_row(result))){
//			ros::Time T_temp;
//			temp_tms_data.tMeasuredTime = T_temp.fromNSec(strtoll(row[0], &endptr, 10));
			temp_tms_data.tMeasuredTime.fromNSec(strtoll(row[0], &endptr, 10));
			temp_tms_data.iID = atoi(row[1]);
			temp_tms_data.iType = 3;
			temp_tms_data.fX = (float)(atof(row[2]));
			temp_tms_data.fY = (float)(atof(row[3]));
			temp_tms_data.fZ = (float)(atof(row[4]));
			temp_tms_data.fTheta = (float)(atof(row[5]));
			temp_tms_data.iPlace = atoi(row[7]);
			temp_tms_data.iState = atoi(row[6]);

			//responseへpush_back
			res.msgTMSInfo.push_back(temp_tms_data);
			count++;
		}
	}
	else{
		printf("itype error\n");
	}

//	for(unsigned int k=0;k<res.msgTMSInfo.size();k++){
//		temp_nsec = res.msgTMSInfo[k].tMeasuredTime.toNSec();
//		std::cout << temp_nsec << ", " << res.msgTMSInfo[k].iType << ", "<< res.msgTMSInfo[k].iID << ", ";
//		std::cout << res.msgTMSInfo[k].fX << ", "<< res.msgTMSInfo[k].fY << ", "<< res.msgTMSInfo[k].fZ;
//		std::cout << res.msgTMSInfo[k].iPlace << ", "<< res.msgTMSInfo[k].iState << std::endl;
//	}

	return true;
}

//by hashiguchi
bool tmsdb_get_task_list_func1(tms_msg_db::tmsdb_get_task_list::Request &req, tms_msg_db::tmsdb_get_task_list::Response &res){

	char query[1024];
	MYSQL_ROW tmp_row;
	char *endptr;

	sprintf(query, "SELECT * FROM tmsdb.task_lists WHERE task_id=%d;", req.task_id);
	printf("%s\n",query);
	if (mysql_query(conn, query)) {
		fprintf(stderr, "%s\n", mysql_error(conn));
		printf("Write error!(person_behavior_query)\n");
		printf("%s\n",query);
	}
	result = mysql_use_result(conn);

	//一意に引き出してくる情報が決まるならこれでよい
	//ない場合はここでエラーを吐くので注意
	//row = mysql_fetch_row(result);
	while((row = mysql_fetch_row(result))){

	tmp_row = row;
	//task_id, task_name, priority, time, sub_task, robot_id, sensor, robot_func, sensor_func
	printf("%s,%s,%s,%s,%s,%s,%s,%s,%s\n",tmp_row[0],tmp_row[1],tmp_row[2],tmp_row[3],tmp_row[4],tmp_row[5],tmp_row[6], tmp_row[7], tmp_row[8]);

//	res.task_id = atoi(tmp_row[0]);
	res.task_name = tmp_row[1];
	res.priority = atoi(tmp_row[2]);
	res.time = strtoll(tmp_row[3], &endptr, 10);
	res.sub_task = tmp_row[4];
	res.robot_id = atoi(tmp_row[5]);
	res.sensor = tmp_row[6];
	res.robot_func = tmp_row[7];
	res.sensor_func = tmp_row[8];
	}

	mysql_free_result(result);

	return true;
}

//by hashiguchi
bool tmsdb_get_unknown_object_info_func1(tms_msg_db::tmsdb_get_unknown_object::Request &req, tms_msg_db::tmsdb_get_unknown_object::Response &res){

	char query[1024];
	MYSQL_ROW tmp_row;

	sprintf(query, "SELECT x,y,z FROM tmsdb.unknown_objects WHERE id=%d", req.unknown_object_id);
	mysql_query(conn, query);
	result = mysql_use_result(conn);
	row = mysql_fetch_row(result);

	tmp_row = row;
	printf("%s,%s,%s\n",tmp_row[0],tmp_row[1],tmp_row[2]);
	res.unknown_object_x = (float)atof(tmp_row[0]);
	res.unknown_object_y = (float)atof(tmp_row[1]);
	res.unknown_object_z = (float)atof(tmp_row[2]);

	mysql_free_result(result);

	return true;
}

//conn reset
bool connRestart(){
	mysql_close(conn);

	conn = mysql_init(NULL);
	if (!mysql_real_connect(conn, DBHOST, DBUSER, DBPASS, DBNAME, 3306, NULL, 0)) {
		fprintf(stderr, "%s\n", mysql_error(conn));
		exit(1);
		return false;
	}

	return true;
}

//RS get object information
bool tms_rs_get_object_info_ex_func(tms_msg_rp::tms_rs_get_object_info_ex::Request &req, tms_msg_rp::tms_rs_get_object_info_ex::Response &res){
	char query[1024];
	MYSQL_ROW tmp_row;

	std::vector<int> objIdArr;
	int index = 0;

	printf("tms_rs_get_object_info_ex_func In!\n");

	sprintf(query, "SELECT id,place,x,y,z,theta FROM tmsdb.furnitures WHERE place!=0 group BY id;");
	mysql_query(conn, query);
	result = mysql_use_result(conn);

	while((row = mysql_fetch_row(result))){
		tmp_row = row;

		res.object_id.push_back(atoi(tmp_row[0]));
		objIdArr.resize(index+1);
		objIdArr[index] = atoi(tmp_row[0]);
		res.place_id.push_back(atoi(tmp_row[1]));
		res.x.push_back(atoi(tmp_row[2]));
		res.y.push_back(atoi(tmp_row[3]));
		res.z.push_back(atoi(tmp_row[4]));
		res.theta.push_back(atoi(tmp_row[5]));
		res.type.push_back(2);
		res.state.push_back(1);
		index++;

	}

	printf("furniture set ended!\n");

	connRestart();

	sprintf(query, "SELECT id,place,x,y,z,theta,state FROM tmsdb.objects_present WHERE place!=0 group BY id;");
	mysql_query(conn, query);
	result = mysql_use_result(conn);

	while((row = mysql_fetch_row(result))){
		tmp_row = row;

		res.object_id.push_back(atoi(tmp_row[0]));
		objIdArr.resize(index+1);
		objIdArr[index] = atoi(tmp_row[0]);
		res.place_id.push_back(atoi(tmp_row[1]));
		res.x.push_back(atoi(tmp_row[2]));
		res.y.push_back(atoi(tmp_row[3]));
		res.z.push_back(atoi(tmp_row[4]));
		res.theta.push_back(atoi(tmp_row[5]));
		res.type.push_back(5);
		res.state.push_back(atoi(tmp_row[6]));
		index++;

	}

	printf("object set ended!\n");

	connRestart();

	//nameの引き出し、セット
	for(int i=0;i<objIdArr.size();i++){
		sprintf(query, "SELECT name FROM tmsdb.id_name WHERE id=%d;",objIdArr[i]);
		printf("%d\n",objIdArr[i]);
		mysql_query(conn, query);
		result = mysql_use_result(conn);

		if ((row = mysql_fetch_row(result)) != NULL){
			tmp_row = row;
			res.name.push_back(tmp_row[0]);
		}

		connRestart();
	}

	printf("name set ended!\n");

	return true;
}

/*
//id -> around object data(name,id)
bool tms_rs_get_around_object_func(tms_msg_rp::tms_rs_get_around_object::Request &req, tms_msg_rp::tms_rs_get_around_object::Response &res){
	char query[1024];
	MYSQL_ROW tmp_row;

	std::vector<int> objIdArr;
	int index = 0;

	//src_idセット
	sprintf(query, "SELECT place FROM tmsdb.furnitures WHERE id=%d UNION SELECT place FROM tmsdb.objects_present WHERE id=%d;",req.src_object_id,req.src_object_id);
	mysql_query(conn, query);
	result = mysql_use_result(conn);

	if((row = mysql_fetch_row(result)) != NULL){
		tmp_row = row;
		objIdArr.resize(index+1);
		objIdArr[index] = atoi(tmp_row[0]);
		res.src_id = atoi(tmp_row[0]);
		index++;
	}

	connRestart();

	//object_id
	sprintf(query, "SELECT id FROM tmsdb.furnitures WHERE place=%d UNION SELECT id FROM tmsdb.objects_present WHERE place=%d;",req.src_object_id,req.src_object_id);
	mysql_query(conn, query);
	result = mysql_use_result(conn);
	while((row = mysql_fetch_row(result))){
		tmp_row = row;
		objIdArr.resize(index+1);
		objIdArr[index] = atoi(tmp_row[0]);
		res.object_id.push_back(atoi(tmp_row[0]));
		index++;
	}

	connRestart();

	//nameの引き出し、セット
	sprintf(query, "SELECT name FROM tmsdb.id_name WHERE id=%d;",objIdArr[0]);
		mysql_query(conn, query);
		result = mysql_use_result(conn);
		if ((row = mysql_fetch_row(result)) != NULL){
		tmp_row = row;
		res.src_name = tmp_row[0];
	}

	connRestart();

	for(int i=1;i<objIdArr.size();i++){
		sprintf(query, "SELECT name FROM tmsdb.id_name WHERE id=%d;",objIdArr[i]);
		mysql_query(conn, query);
		result = mysql_use_result(conn);

		if ((row = mysql_fetch_row(result)) != NULL){
			tmp_row = row;
			res.name.push_back(tmp_row[0]);
			int type = 0;
			if(objIdArr[i] >= 50) type = 5;
			res.type.push_back(type);
		}

		connRestart();
	}


	return true;
}
*/
/*
//id -> object infomation
bool tms_rs_get_object_info_rt_func(tms_msg_rp::tms_rs_get_object_info_rt::Request &req, tms_msg_rp::tms_rs_get_object_info_rt::Response &res){
	char query[1024];
	MYSQL_ROW tmp_row;
	int objIdArr[2] = {0,0};

	sprintf(query, "SELECT id,place,x,y,z,theta FROM tmsdb.furnitures WHERE id=%d UNION SELECT id,place,x,y,z,theta FROM tmsdb.objects_present WHERE id=%d;",req.request,req.request);
	mysql_query(conn, query);
	result = mysql_use_result(conn);

	if((row = mysql_fetch_row(result))){
		tmp_row = row;
		res.object_id = atoi(tmp_row[0]);
		res.place_id = atoi(tmp_row[1]);
		objIdArr[0] = atoi(tmp_row[0]);
		objIdArr[1] = atoi(tmp_row[1]);
		res.x = atoi(tmp_row[2]);
		res.y = atoi(tmp_row[3]);
		res.z = atoi(tmp_row[4]);
		res.theta = atoi(tmp_row[5]);
		int type = 0;
		if(res.object_id>=50) type = 5;
		res.type = type;
		res.state = 1;
	}

	connRestart();

	//nameの引き出し、セット
	sprintf(query, "SELECT name FROM tmsdb.id_name WHERE id=%d;",objIdArr[0]);
	mysql_query(conn, query);
	result = mysql_use_result(conn);

	if ((row = mysql_fetch_row(result)) != NULL){
		tmp_row = row;
		res.name = tmp_row[0];
	}

	connRestart();

	sprintf(query, "SELECT name FROM tmsdb.id_name WHERE id=%d;",objIdArr[1]);
	mysql_query(conn, query);
	result = mysql_use_result(conn);

	if ((row = mysql_fetch_row(result)) != NULL){
		tmp_row = row;
		res.place_name = tmp_row[0];
	}

	connRestart();

	return true;
}
*/


int main(int argc, char **argv){

	int count = 0;
	char query[1024];

	if ((fp = fopen("test.log", "w")) == NULL) {
		printf("file open error!!\n");
		exit(EXIT_FAILURE);
	}

	ros::init(argc, argv, "db_reader");
	ros::NodeHandle n;
	printf("db_reader : init\n");

	// MySQL connection
	conn = mysql_init(NULL);
	if (!mysql_real_connect(conn, DBHOST, DBUSER, DBPASS, DBNAME, 3306, NULL, 0)) {
		fprintf(stderr, "%s\n", mysql_error(conn));
		exit(1);
	}
	printf("MySQL opened\n");
	
//	ros::ServiceServer service1 = n.advertiseService("tmsdb_planning", get_planning_info);
	ros::ServiceServer service2 = n.advertiseService("tmsdb_get_objects_info_present", tmsdb_get_objects_info_func1);
	ros::ServiceServer service3 = n.advertiseService("tmsdb_get_objects_info_history", tmsdb_get_objects_info_func2);

	ros::ServiceServer service4 = n.advertiseService("tmsdb_get_furnitures_info", tmsdb_get_furnitures_info_func1);
//	ros::ServiceServer service4 = n.advertiseService("tmsdb_object_detection_furniture", get_obj_dt_info);
	ros::ServiceServer service5 = n.advertiseService("tmsdb_get_pcd_info", tmsdb_get_pcd_info_func1);
	ros::ServiceServer service6 = n.advertiseService("tmsdb_get_movable_furnitures_info", tmsdb_get_movable_furnitures_info_func1);
	ros::ServiceServer service7 = n.advertiseService("tmsdb_robot_trajectory_data", tmsdb_robot_trajectory_data);
	ros::ServiceServer service8 = n.advertiseService("tmsdb_get_robots_info", tmsdb_get_robots_info_func1);
//	ros::ServiceServer service9 = n.advertiseService("tmsdb_read_robots_history", tmsdb_read_robots_history);
	ros::ServiceServer service10 = n.advertiseService("tmsdb_get_person_info", tmsdb_get_person_info_func1);
	ros::ServiceServer service11 = n.advertiseService("tmsdb_get_person_info_2", tmsdb_get_person_info_func2);
	ros::ServiceServer service12 = n.advertiseService("tmsdb_get_person_info_3", tmsdb_get_person_info_func3);
  //by hashiguchi
	ros::ServiceServer service13 = n.advertiseService("tmsdb_get_task_list", tmsdb_get_task_list_func1);
	ros::ServiceServer service14 = n.advertiseService("tmsdb_get_unknown_object_info", tmsdb_get_unknown_object_info_func1);
  // by pyo
  ros::ServiceServer service20 = n.advertiseService("tmsdb_get_current_objects_info", tmsdb_get_current_objects_info);
	ros::ServiceServer service21 = n.advertiseService("tmsdb_get_current_person_info", tmsdb_get_current_person_info);
  ros::ServiceServer service22 = n.advertiseService("tmsdb_get_current_robot_info", tmsdb_get_current_robot_info);
  ros::ServiceServer service23 = n.advertiseService("tms_rs_get_object_info_ex", tms_rs_get_object_info_ex_func);
  //ros::ServiceServer service24 = n.advertiseService("tms_rs_get_around_object", tms_rs_get_around_object_func);
  //ros::ServiceServer service25 = n.advertiseService("tms_rs_get_object_info_rt", tms_rs_get_object_info_rt_func);

	//タイマーを利用して定期的に取得すべきかもしれない
	count = 0;
	sprintf(query, "SELECT id,name,type FROM tmsdb.id_name ORDER BY id;");
	printf("%s\n", query);
	mysql_query(conn, query);
	result = mysql_use_result(conn);
	while((row = mysql_fetch_row(result))){
		id_name_itype[atoi(row[0])].id = (atoi(row[0]));
		strcpy(id_name_itype[atoi(row[0])].name, row[1]);
		id_name_itype[atoi(row[0])].type = (atoi(row[2]));
		count++;
	}
	for(int i=0;i<5;i++){
		printf("id_name_itype[%3d]: %d %s %d\n", i, id_name_itype[i].id, id_name_itype[i].name, id_name_itype[i].type);
	}

	ros::spin();

	// close connection
//	mysql_free_result(result);
	mysql_close(conn);

	return 0;
}
