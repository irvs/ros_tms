//------------------------------------------------------------------------------
// @file   : tms_db_writer.cpp
// @brief  : write data into rostms database
// @author : Yoonseok Pyo
// @version: Ver0.2.2 (since 2014.05.07)
// @date   : 2014.11.19
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

#define STRUCTURE_ID      4000
#define MAX_STRUCTURE_NUM 1
#define SPACE_ID          5000
#define MAX_SPACE_NUM     4
#define FURNITURE_ID      6000
#define MAX_FURNITURE_NUM 20

//------------------------------------------------------------------------------
using std::string;

//------------------------------------------------------------------------------
class DbWriter
{
//------------------------------------------------------------------------------
private:
  // ROS NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // ROS Topic Subscriber
  ros::Subscriber     data_sub;
  ros::ServiceClient  get_data_client;
  // ROS Parameters:
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

//------------------------------------------------------------------------------
public:
  DbWriter() : 
    nh_priv("~"),
    dbhost("192.168.4.170"),
    dbuser("root"),
    dbpass("tmsdb"),
    dbname("rostmsdb"),
    is_debug(false)
  {
    //Init parameter
    nh_priv.param("is_debug", is_debug, is_debug);
    //Init Vicon Stream
    ROS_ASSERT(initDbWriter());
    // Subscriber for tms_db_data topic
    data_sub = nh.subscribe("tms_db_data", 100, &DbWriter::dbWriteCallback, this);
    writeInitData();
  }  

  //----------------------------------------------------------------------------
  ~DbWriter()
  {
    ROS_ASSERT(shutdownDbWriter());
  } 

//------------------------------------------------------------------------------
private:
  bool initDbWriter()
  {
    ROS_INFO("tms_db_writer : Init OK!\n");

    //Connection to a MySQL database 
    connector = mysql_init(NULL);
    if (!mysql_real_connect(connector,
                            dbhost.c_str(),
                            dbuser.c_str(),
                            dbpass.c_str(),
                            dbname.c_str(),
                            3306,
                            NULL, CLIENT_MULTI_STATEMENTS)) {
      fprintf(stderr, "%s\n", mysql_error(connector));
      return false;
    }

    ROS_INFO("MySQL(rostmsdb) opened.\n");
    return true;
  }

  //----------------------------------------------------------------------------
  bool shutdownDbWriter()
  {
    //Close connection
    mysql_close(connector);
    ROS_INFO("MySQL(rostmsdb) closed.\n");
    return true;
  }

  //----------------------------------------------------------------------------
  bool writeInitData()
  {
    tms_msg_db::Tmsdb temp_dbdata;

    writeData(2008);  // ardrone
    writeData(2009);  // refrigerator

    for(int32_t i=1; i <= MAX_STRUCTURE_NUM; i++)
    {
      writeData(STRUCTURE_ID + i);
    }

    for(int32_t i=1; i <= MAX_SPACE_NUM; i++)
    {
      writeData(SPACE_ID + i);
    }

    for(int32_t i=1; i <= MAX_FURNITURE_NUM; i++)
    {
      writeData(FURNITURE_ID + i);
    }

    return true;
  }

  //----------------------------------------------------------------------------
  bool writeData(int32_t id)
  {
    tms_msg_db::Tmsdb temp_dbdata;
    ros::Time now;
    char select_query[1024];
    char insert_query[1024];
    char delete_query[1024];

    sprintf(select_query, "SELECT * FROM rostmsdb.id WHERE id=%d;", id);
    if(is_debug) ROS_INFO("%s\n", select_query);
    mysql_query(connector, select_query);
    result = mysql_use_result(connector);
    while ((row = mysql_fetch_row(result)) != NULL)
    {
      for(int32_t j=0;j<25;j++) if(is_debug) ROS_INFO("%s, ",row[j]);
      now = ros::Time::now() + ros::Duration(9*60*60); // GMT +9
      temp_dbdata.time        = boost::posix_time::to_iso_extended_string(now.toBoost());
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
    }
    if(mysql_num_rows(result) != 0)
      mysql_free_result(result);

    //------------------------------------------------------------------------
    // Delete old data in rostmsdb.data_xxx
    sprintf(delete_query,
            "DELETE FROM rostmsdb.data_%s WHERE time <'%s' AND id=%d AND sensor=%d;",
            temp_dbdata.type.c_str(),
            temp_dbdata.time.c_str(),
            temp_dbdata.id,
            temp_dbdata.sensor);

    if(is_debug) ROS_INFO("%s\n", delete_query);

    if (mysql_query(connector, delete_query))
    {
      ROS_ERROR("%s", mysql_error(connector));
      ROS_ERROR("DB write error!");
    }
    
    //------------------------------------------------------------------------
    // Insert new data to rostmsdb.data_xxx
    sprintf(insert_query,
      "INSERT INTO rostmsdb.data_%s VALUES ('%s','%s',%d,'%s',%f,%f,%f,%f,%f,%f,%f,%f,%f,'%s',%f,'%s','%s',%d,'%s',%d,%f,%d,'%s','%s','%s');",
      temp_dbdata.type.c_str(),
      temp_dbdata.time.c_str(),
      temp_dbdata.type.c_str(),
      temp_dbdata.id,
      temp_dbdata.name.c_str(),
      temp_dbdata.x,
      temp_dbdata.y,
      temp_dbdata.z,
      temp_dbdata.rr,
      temp_dbdata.rp,
      temp_dbdata.ry,
      temp_dbdata.offset_x,
      temp_dbdata.offset_y,
      temp_dbdata.offset_z,
      temp_dbdata.joint.c_str(),
      temp_dbdata.weight,
      temp_dbdata.rfid.c_str(),
      temp_dbdata.etcdata.c_str(),
      temp_dbdata.place,
      temp_dbdata.extfile.c_str(),
      temp_dbdata.sensor,
      temp_dbdata.probability,
      temp_dbdata.state,
      temp_dbdata.task.c_str(),
      temp_dbdata.note.c_str(),
      temp_dbdata.tag.c_str());

    if(is_debug) ROS_INFO("%s\n",insert_query);

    if (mysql_query(connector, insert_query))
    {
      fprintf(stderr, "%s\n", mysql_error(connector));
      ROS_ERROR("Write error!\n");
    }
    
    return true;
  }

  //----------------------------------------------------------------------------
  // dbWriteCallback function
  void dbWriteCallback(const tms_msg_db::TmsdbStamped::ConstPtr& msg)
  {
    int32_t msg_size;
    char select_query[1024];
    char insert_query[1024];
    char delete_query[1024];
    MYSQL_ROW   temp_row;
    std::string temp_type;
    std::string temp_name;
    std::string temp_probability;

    msg_size = msg->tmsdb.size();
    if(is_debug) ROS_INFO("msg_size = %d\n", msg_size);
    for(int32_t i=0; i<msg_size; i++)
    {
      // Search the type, name, etc infomation in ID table
      sprintf(select_query, "SELECT type,name,probability FROM rostmsdb.id WHERE id=%d", msg->tmsdb[i].id);
      if(is_debug) ROS_INFO("%s\n", select_query);
      mysql_query(connector, select_query);
      result   = mysql_use_result(connector);
      row      = mysql_fetch_row(result);
      temp_row = row;
      if(is_debug) ROS_INFO("%s,%s\n",temp_row[0],temp_row[1]);
      temp_type         = temp_row[0];
      temp_name         = temp_row[1];
      temp_probability  = 0.0;
      mysql_free_result(result);

      if (msg->tmsdb[i].sensor != 0)
      {
        // Search the sensor's probability infomation in ID table
        sprintf(select_query, "SELECT probability FROM rostmsdb.id WHERE id=%d", msg->tmsdb[i].sensor);
        if(is_debug) ROS_INFO("%s\n", select_query);
        mysql_query(connector, select_query);
        result  = mysql_use_result(connector);
        row     = mysql_fetch_row(result);
        temp_row = row;
        if(is_debug) ROS_INFO("%s\n",temp_row[0]);
        temp_probability  = temp_row[0]==NULL?"0.0":temp_row[0];
        mysql_free_result(result);
      }

      //------------------------------------------------------------------------
      // Insert new data to rostmsdb.history_data
      sprintf(insert_query, 
        "INSERT INTO rostmsdb.history_data VALUES ('%s','%s',%d,'%s',%f,%f,%f,%f,%f,%f,%f,%f,%f,'%s',%f,'%s','%s',%d,'%s',%d,%f,%d,'%s','%s','%s');",
        msg->tmsdb[i].time.c_str(),
        msg->tmsdb[i].type.empty()?temp_type.c_str():msg->tmsdb[i].type.c_str(),
        msg->tmsdb[i].id,
        msg->tmsdb[i].name.empty()?temp_name.c_str():msg->tmsdb[i].name.c_str(),
        msg->tmsdb[i].x,
        msg->tmsdb[i].y,
        msg->tmsdb[i].z,
        msg->tmsdb[i].rr,
        msg->tmsdb[i].rp,
        msg->tmsdb[i].ry,
        msg->tmsdb[i].offset_x,
        msg->tmsdb[i].offset_y,
        msg->tmsdb[i].offset_z,
        msg->tmsdb[i].joint.empty()?"":msg->tmsdb[i].joint.c_str(),
        msg->tmsdb[i].weight,
        msg->tmsdb[i].rfid.empty()?"":msg->tmsdb[i].rfid.c_str(),
        msg->tmsdb[i].etcdata.empty()?"":msg->tmsdb[i].etcdata.c_str(),
        msg->tmsdb[i].place,
        msg->tmsdb[i].extfile.empty()?"":msg->tmsdb[i].extfile.c_str(),
        msg->tmsdb[i].sensor,
        atof(temp_probability.c_str()),
        msg->tmsdb[i].state,
        msg->tmsdb[i].task.empty()?"":msg->tmsdb[i].task.c_str(),
        msg->tmsdb[i].note.empty()?"":msg->tmsdb[i].note.c_str(),
        msg->tmsdb[i].tag.empty()?"":msg->tmsdb[i].tag.c_str());

      if(is_debug) ROS_INFO("%s\n",insert_query);

      if (mysql_query(connector, insert_query))
      {
        fprintf(stderr, "%s\n", mysql_error(connector));
        ROS_INFO("Write error!\n");
      }

      //------------------------------------------------------------------------
      // Delete old data in rostmsdb.data_xxx
      sprintf(delete_query,
              "DELETE FROM rostmsdb.data_%s WHERE time <'%s' AND id=%d AND sensor=%d;",
              temp_type.c_str(),
              msg->tmsdb[i].time.c_str(),
              msg->tmsdb[i].id,
              msg->tmsdb[i].sensor);

      if(is_debug) ROS_INFO("%s\n", delete_query);

      if (mysql_query(connector, delete_query))
      {
        ROS_ERROR("%s", mysql_error(connector));
        ROS_ERROR("DB write error!");
      } 

      //------------------------------------------------------------------------
      // Insert new data to rostmsdb.data_xxx
      sprintf(insert_query, 
        "INSERT INTO rostmsdb.data_%s VALUES ('%s','%s',%d,'%s',%f,%f,%f,%f,%f,%f,%f,%f,%f,'%s',%f,'%s','%s',%d,'%s',%d,%f,%d,'%s','%s','%s');",
        temp_type.c_str(),
        msg->tmsdb[i].time.c_str(),
        msg->tmsdb[i].type.empty()?temp_type.c_str():msg->tmsdb[i].type.c_str(),
        msg->tmsdb[i].id,
        msg->tmsdb[i].name.empty()?temp_name.c_str():msg->tmsdb[i].name.c_str(),
        msg->tmsdb[i].x,
        msg->tmsdb[i].y,
        msg->tmsdb[i].z,
        msg->tmsdb[i].rr,
        msg->tmsdb[i].rp,
        msg->tmsdb[i].ry,
        msg->tmsdb[i].offset_x,
        msg->tmsdb[i].offset_y,
        msg->tmsdb[i].offset_z,
        msg->tmsdb[i].joint.empty()?"":msg->tmsdb[i].joint.c_str(),
        msg->tmsdb[i].weight,
        msg->tmsdb[i].rfid.empty()?"":msg->tmsdb[i].rfid.c_str(),
        msg->tmsdb[i].etcdata.empty()?"":msg->tmsdb[i].etcdata.c_str(),
        msg->tmsdb[i].place,
        msg->tmsdb[i].extfile.empty()?"":msg->tmsdb[i].extfile.c_str(),
        msg->tmsdb[i].sensor,
        atof(temp_probability.c_str()),
        msg->tmsdb[i].state,
        msg->tmsdb[i].task.empty()?"":msg->tmsdb[i].task.c_str(),
        msg->tmsdb[i].note.empty()?"":msg->tmsdb[i].note.c_str(),
        msg->tmsdb[i].tag.empty()?"":msg->tmsdb[i].tag.c_str());

      if(is_debug) ROS_INFO("%s\n",insert_query);

      if (mysql_query(connector, insert_query))
      {
        fprintf(stderr, "%s\n", mysql_error(connector));
        ROS_INFO("Write error!\n");
      } 
    }
  }
};

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //Init ROS node
  ros::init(argc, argv, "tms_db_writer");
  DbWriter dw;
  ros::spin();
  return 0;
}

//------------------------------------------------------------------------------
//EOF
