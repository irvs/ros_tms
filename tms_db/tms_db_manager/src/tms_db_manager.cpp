//------------------------------------------------------------------------------
// @file   : tms_db_manager.cpp
// @brief  : manage the ros-tms database
// @author : Yoonseok Pyo
// @version: Ver0.2.0 (since 2014.05.07)
// @date   : 2015.08.26
//------------------------------------------------------------------------------
//include for ROS
#include <ros/ros.h>
#include <unistd.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_db/Tmsdb.h>

//include for Mysql
#include <mysql/mysql.h>

//include for std
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>

//------------------------------------------------------------------------------
using std::string;
using std::vector;

//------------------------------------------------------------------------------
class DbManager
{
//------------------------------------------------------------------------------
private:
  // ROS NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // ROS Timer
  ros::Timer update_timer;
  ros::Timer remove_timer;
  // ROS Parameters:
  double update_time;
  bool is_debug;
  // MySQL structures
  // https://dev.mysql.com/doc/refman/5.6/en/c-api-data-structures.html
  MYSQL     *connector;
  MYSQL_RES *result;
  MYSQL_ROW row;
  int32_t   fields;
  // MySQL information
  string dbhost;
  string dbuser;
  string dbpass;
  string dbname;
  string dbdata[100];

//------------------------------------------------------------------------------
public:
  DbManager() :
    nh_priv("~"),
    dbhost("192.168.4.106"),
    dbuser("root"),
    dbpass("tmsdb"),
    dbname("rostmsdb"),
    update_time(60),  //sec
    is_debug(false)
  {
    //Init parameter
    nh_priv.param("update_time", update_time, update_time);
    nh_priv.param("is_debug", is_debug, is_debug);
    //Init Vicon Stream
    ROS_ASSERT(initDbManager());
    //TimerEvent
    update_timer = nh.createTimer(ros::Duration(update_time), &DbManager::manageDataCallback, this);
    remove_timer = nh.createTimer(ros::Duration(12*60*60), &DbManager::removeForeverDataCallback, this);
  }

  //----------------------------------------------------------------------------
  ~DbManager()
  {
    ROS_ASSERT(shutdownDbManager());
  }

//------------------------------------------------------------------------------
private:
  bool initDbManager()
  {
    ROS_INFO("tms_db_manager : Init OK!\n");

    //Connection to a MySQL database
    connector = mysql_init(NULL);
    if (!mysql_real_connect(connector, dbhost.c_str(), dbuser.c_str(), dbpass.c_str(), dbname.c_str(), 3306, NULL, CLIENT_MULTI_STATEMENTS))
    {
      fprintf(stderr, "%s\n", mysql_error(connector));
      return false;
    }

    ROS_INFO("MySQL(rostmsdb) opened.\n");
    return true;
  }

  //----------------------------------------------------------------------------
  bool shutdownDbManager()
  {
    //Close connection
    mysql_close(connector);
    ROS_INFO("MySQL(rostmsdb) closed.\n");
    return true;
  }

  //----------------------------------------------------------------------------
  bool getIdList(string temp_type, string *idList)
  {
    // Search the id information using type information in ID table
    char select_query[1024];
    sprintf(select_query, "SELECT id FROM rostmsdb.id WHERE type='%s'", temp_type.c_str());
    if(is_debug) ROS_INFO("%s\n", select_query);
    mysql_query(connector, select_query);
    result  = mysql_store_result(connector);
    while ((row = mysql_fetch_row(result)) != NULL)
    {
      *idList++ = row[0];
    }
    if(mysql_num_rows(result) != 0)
      mysql_free_result(result);

    return true;
  }

  //----------------------------------------------------------------------------
  bool storeBackupData()
  {
    char delete_query[1024];
    char insert_query[1024];

    ros::Time update_period = ros::Time::now() + ros::Duration(9*60*60) - ros::Duration(1*60*60); // GMT +9 -1
    string iso_update_period = boost::posix_time::to_iso_extended_string(update_period.toBoost());

    //------------------------------------------------------------------------
    // Insert old data in history_data table into the backup_data table
    sprintf(insert_query,
            "INSERT INTO rostmsdb.backup_data SELECT * FROM rostmsdb.history_data WHERE time <'%s';",
            iso_update_period.c_str());

    if(is_debug) ROS_INFO("%s\n", insert_query);

    if (mysql_query(connector, insert_query))
    {
      ROS_ERROR("%s", mysql_error(connector));
      ROS_ERROR("DB write error!");
    }

    //------------------------------------------------------------------------
    // Delete old data in history_data table
    sprintf(delete_query,
            "DELETE FROM rostmsdb.history_data WHERE time <'%s';",
            iso_update_period.c_str());

    if(is_debug) ROS_INFO("%s\n", delete_query);

    if (mysql_query(connector, delete_query))
    {
      ROS_ERROR("%s", mysql_error(connector));
      ROS_ERROR("DB write error!");
    }

    return true;
  }

  //----------------------------------------------------------------------------
  void manageDataCallback(const ros::TimerEvent& e)
  {
    nh_priv.getParam("is_debug", is_debug);

    if(is_debug) ROS_INFO("manageDataCallback triggered");

    storeBackupData();
  }

  //----------------------------------------------------------------------------
  void removeForeverDataCallback(const ros::TimerEvent& e)
  {
    nh_priv.getParam("is_debug", is_debug);

    if(is_debug) ROS_INFO("removeForeverDataCallback triggered");

    char delete_query[1024];

    ros::Time update_period = ros::Time::now() + ros::Duration(9*60*60) - ros::Duration(14*24*60*60); // 14day (GMT +9)
    string iso_update_period = boost::posix_time::to_iso_extended_string(update_period.toBoost());

    //------------------------------------------------------------------------
    // Delete old data in backup_data table
    sprintf(delete_query,
            "DELETE FROM rostmsdb.backup_data WHERE time <'%s';",
            iso_update_period.c_str());

    if(is_debug) ROS_INFO("%s\n", delete_query);

    if (mysql_query(connector, delete_query))
    {
      ROS_ERROR("%s", mysql_error(connector));
      ROS_ERROR("DB write error!");
    }

    return;
  }
};

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //Init ROS node
  ros::init(argc, argv, "tms_db_manager");
  DbManager dm;
  ros::spin();
  return 0;
}

//------------------------------------------------------------------------------
//EOF
