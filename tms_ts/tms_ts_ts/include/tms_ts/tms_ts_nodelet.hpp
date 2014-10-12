/*
 * tms_ts_nodelet.hpp
 *
 *  Created on: 2013/10/01
 *      Author: hashiguchi
 */

#ifndef TMS_TS_NODELET_HPP_
#define TMS_TS_NODELET_HPP_

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <assert.h>

#include <sstream>
#include <fstream>
#include <boost/date_time/local_time/local_time.hpp>
#include <time.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <tms_msg_ts/ts_req.h> // from UR
#include <tms_msg_db/TmsdbGetData.h> // to DB

namespace tms_ts_nodelet
{

class ROS_TMS_TS : public nodelet::Nodelet
{
public:
  ROS_TMS_TS() : robot_id(0), object_id(0), user_id(0), place_id(0)
  {}

private:
  virtual void onInit();

  std::string rosCheckTime(boost::posix_time::ptime time);

  // スクリプト生成関数
  void GenerateScript(void);
  // スクリプト実行関数(smach)
  bool ExeScript(void);

  // コールバック関数(from ts_master)
  bool tsCallback(tms_msg_ts::ts_req::Request &req, tms_msg_ts::ts_req::Response &res);

  // リクエスト情報格納用変数
  int robot_id;
  int object_id;
  int user_id;
  int place_id;

  // タスク・サブタスク情報格納用変数
  struct SubTask {
	  std::string subtask_id;
	  std::string subtask_name;
	  std::vector<double> arg;
	  //int arg;
  };
  std::vector<SubTask> sub_task;

  // smach	ファイル名
  std::string file_name;

  ros::ServiceServer service; // URとの通信
  ros::ServiceClient db_reader_client; // task_dataを取得するためのTMSDBとの通信
  ros::ServiceClient db_reader_client2; // subtask_dataを取得するためのTMSDBとの通信
};


PLUGINLIB_DECLARE_CLASS(tms_ts_nodelet, ROS_TMS_TS, tms_ts_nodelet::ROS_TMS_TS, nodelet::Nodelet);
}

#endif /* ROS_TMS_TS_NODELET_HPP_ */
