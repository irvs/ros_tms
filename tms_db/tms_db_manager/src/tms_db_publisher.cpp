
//------------------------------------------------------------------------------
// @file   : tms_db_publisher.cpp
// @brief  : read the ros-tms database and publish the current information
// @author : Yoonseok Pyo
// @version: Ver0.0.1 (since 2014.11.24)
// @date   : 2014.11.24
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
#include <map>

#define MODE_ALL            0
#define MODE_NAME_IDTABLE   1
#define MODE_NAME           2
#define MODE_NAME_SENSOR    3
#define MODE_ID_IDTABLE     4
#define MODE_ID             5
#define MODE_ID_SENSOR      6
#define MODE_TYPE_IDTABLE   7
#define MODE_TYPE           8
#define MODE_TYPE_SENSOR    9
#define MODE_PLACE_IDTABLE 10
#define MODE_PLACE         11
#define MODE_PLACE_TYPE    12
#define MODE_HIERARCHY     13
#define MODE_TAG_IDTABLE   14
#define MODE_ERROR        999

//------------------------------------------------------------------------------
using std::string;
using std::vector;

//------------------------------------------------------------------------------
class DbPublisher
{
//------------------------------------------------------------------------------
private:
  // ROS NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // ROS Parameters
  bool is_debug;
  // MySQL structures
  // https://dev.mysql.com/doc/refman/5.6/en/c-api-data-structures.html
  MYSQL     *connector;
  MYSQL_RES *result;
  MYSQL_ROW row;
  // MySQL information
  string dbhost;
  string dbuser;
  string dbpass;
  string dbname;
  string dbdata[100];
  uint32_t sid;
  std::map<uint32_t, string> target;

//------------------------------------------------------------------------------
public:
  // ROS Publisher
  tms_msg_db::TmsdbStamped current_environment_information;
  ros::Publisher data_pub;

  DbPublisher() :
    nh_priv("~"),
    dbhost("192.168.4.170"),
    dbuser("root"),
    dbpass("tmsdb"),
    dbname("rostmsdb"),
    sid(100000),
    is_debug(false)
  {
    //Init parameter
    nh_priv.param("dbhost",   dbhost, dbhost);
    nh_priv.param("dbuser",   dbuser, dbuser);
    nh_priv.param("dbpass",   dbpass, dbpass);
    nh_priv.param("dbname",   dbname, dbname);
    nh_priv.param("is_debug", is_debug, is_debug);
    //Init Taget name
    target[0] = "person";
    target[1] = "robot";
    target[2] = "sensor";
    target[3] = "structure";
    target[4] = "space";
    target[5] = "furniture";
    target[6] = "object";

    ROS_ASSERT(initDbPublisher());

    data_pub = nh_priv.advertise<tms_msg_db::TmsdbStamped>("db_publisher", 10);
  }

  //----------------------------------------------------------------------------
  ~DbPublisher()
  {
    ROS_ASSERT(shutdownDbPublisher());
  }

  //----------------------------------------------------------------------------
  void getDbCurrentInformation()
  {
    nh_priv.getParam("is_debug", is_debug);

    tms_msg_db::Tmsdb temp_dbdata;
    char select_query[1024];

    current_environment_information.tmsdb.clear();;

    //--------------------------------------------------------------------------
    for (uint32_t i = 0; i < 7; i++)
    {
      sprintf(select_query, "SELECT * FROM rostmsdb.data_%s WHERE state!=\"0\";", target[i].c_str());
      if(is_debug) ROS_INFO("%s\n", select_query);

      mysql_query(connector, select_query);
      result = mysql_use_result(connector);

      if (result == NULL)
      {
        temp_dbdata.note = "Wrong request! Try to check the query";
        return;
      }

      while ((row = mysql_fetch_row(result)) != NULL)
      {
        for(int32_t j=0;j<25;j++) if(is_debug) ROS_INFO("%s, ",row[j]);
        temp_dbdata.time        = row[0];
        temp_dbdata.type        = row[1];
        temp_dbdata.id          = atoi(row[2]);
        temp_dbdata.name        = row[3];
        temp_dbdata.x           = atof(row[4]);
        temp_dbdata.y           = atof(row[5]);
        temp_dbdata.z           = atof(row[6]);
        temp_dbdata.rr          = atof(row[7]);
        temp_dbdata.rp          = atof(row[8]);
        temp_dbdata.ry          = atof(row[9]);
        temp_dbdata.offset_x    = atof(row[10]);
        temp_dbdata.offset_y    = atof(row[11]);
        temp_dbdata.offset_z    = atof(row[12]);
        temp_dbdata.joint       = row[13];
        temp_dbdata.weight      = atof(row[14]);
        temp_dbdata.rfid        = row[15];
        temp_dbdata.etcdata     = row[16];
        temp_dbdata.place       = atoi(row[17]);
        temp_dbdata.extfile     = row[18];
        temp_dbdata.sensor      = atoi(row[19]);
        temp_dbdata.probability = atof(row[20]);
        temp_dbdata.state       = atoi(row[21]);
        temp_dbdata.task        = row[22];
        temp_dbdata.note        = row[23];
        temp_dbdata.tag         = row[24];
        current_environment_information.tmsdb.push_back(temp_dbdata);
      }

      if(mysql_num_rows(result) != 0)
        mysql_free_result(result);
    }
    return;
  }

//------------------------------------------------------------------------------
private:
  bool initDbPublisher()
  {
    ROS_INFO("tms_db_publisher : Init OK!\n");

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
  bool shutdownDbPublisher()
  {
    //Close connection
    mysql_close(connector);
    ROS_INFO("MySQL(rostmsdb) closed.\n");
    return true;
  }
};

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //Init ROS node
  ros::init(argc, argv, "tms_db_publisher");
  DbPublisher dp;
  ros::Rate loop_rate(100); // 0.01sec

  while (ros::ok())
  {
    dp.getDbCurrentInformation();
    dp.data_pub.publish(dp.current_environment_information);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

//------------------------------------------------------------------------------
//EOF
