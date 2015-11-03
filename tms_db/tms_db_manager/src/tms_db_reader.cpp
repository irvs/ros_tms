//------------------------------------------------------------------------------
// @file   : tms_db_reader.cpp
// @brief  : read the ros-tms database
// @author : Yoonseok Pyo
// @version: Ver0.2.3 (since 2014.05.09)
// @date   : 2015.08.26
//------------------------------------------------------------------------------
//include for ROS
#include <ros/ros.h>
#include <unistd.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <tms_msg_db/TmsdbGetData.h>
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
class DbReader
{
//------------------------------------------------------------------------------
private:
  // ROS NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // ROS ServiceServer
  ros::ServiceServer dr_srv;
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
  std::map<uint32_t, string> Target;

//------------------------------------------------------------------------------
public:
  DbReader() : 
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
    Target[0] = "person";
    Target[1] = "robot";
    Target[2] = "sensor";
    Target[3] = "structure";
    Target[4] = "space";
    Target[5] = "furniture";
    Target[6] = "object";


    ROS_ASSERT(initDbReader());

    //Service Server
    dr_srv = nh_priv.advertiseService("dbreader",&DbReader::dbSrvCallback, this);
  }  

  //----------------------------------------------------------------------------
  ~DbReader()
  {
    ROS_ASSERT(shutdownDbReader());
  } 

//------------------------------------------------------------------------------
private:
  bool initDbReader()
  {
    ROS_INFO("tms_db_reader : Init OK!\n");

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
  bool shutdownDbReader()
  {
    //Close connection
    mysql_close(connector);
    ROS_INFO("MySQL(rostmsdb) closed.\n");
    return true;
  }

  //----------------------------------------------------------------------------
  bool dbSrvCallback(tms_msg_db::TmsdbGetData::Request& req, tms_msg_db::TmsdbGetData::Response& res)
  {
    nh_priv.getParam("is_debug", is_debug);

    tms_msg_db::Tmsdb temp_dbdata;
    char select_query[1024];
    int32_t     mode = 0;
    uint32_t    temp_id;
    std::string temp_type;
    std::string temp_name;
    uint32_t    temp_place;

    //--------------------------------------------------------------------------
    if(req.tmsdb.tag.empty()==false)
    {
      mode = MODE_TAG_IDTABLE;
    }
    else if (req.tmsdb.id == 0 && req.tmsdb.type.empty()==true && req.tmsdb.sensor == 0 && req.tmsdb.place == 0 && req.tmsdb.name.empty()==true)
    {
      mode = MODE_ALL;
    }
    else if(req.tmsdb.id == sid && req.tmsdb.name.empty()==false)
    {
      mode = MODE_NAME_IDTABLE;
    }
    else if (req.tmsdb.id == 0 && req.tmsdb.name.empty()==false && req.tmsdb.id != sid && req.tmsdb.type.empty()==true && req.tmsdb.sensor == 0 && req.tmsdb.place == 0)
    {
      mode = MODE_NAME; 
    }
    else if (req.tmsdb.id == 0 && req.tmsdb.name.empty()==false && req.tmsdb.id != sid && req.tmsdb.type.empty()==true && req.tmsdb.sensor != 0 && req.tmsdb.place == 0)
    {
      mode = MODE_NAME_SENSOR;       
    }
    else if(req.tmsdb.id > sid && req.tmsdb.type.empty()==true)
    {
      req.tmsdb.id -= sid;
      mode = MODE_ID_IDTABLE; 
    }
    else if (req.tmsdb.id != 0 && req.tmsdb.id != sid && req.tmsdb.type.empty()==true && req.tmsdb.sensor == 0 && req.tmsdb.place == 0)
    {
      mode = MODE_ID; 
    }
    else if (req.tmsdb.id != 0 && req.tmsdb.id != sid && req.tmsdb.type.empty()==true && req.tmsdb.sensor != 0 && req.tmsdb.place == 0)
    {
      mode = MODE_ID_SENSOR; 
    }
    else if (req.tmsdb.id == sid && req.tmsdb.type.empty()==false)
    {
      mode = MODE_TYPE_IDTABLE; 
    }
    else if (req.tmsdb.id == 0 && req.tmsdb.type.empty()==false && req.tmsdb.sensor == 0 && req.tmsdb.place ==0)
    {
      mode = MODE_TYPE; 
    }
    else if (req.tmsdb.id == 0 && req.tmsdb.type.empty()==false && req.tmsdb.sensor != 0 && req.tmsdb.place ==0)
    {
      mode = MODE_TYPE_SENSOR; 
    }
    else if (req.tmsdb.id == sid && req.tmsdb.type.empty()==true && req.tmsdb.place !=0)
    {
      mode = MODE_PLACE_IDTABLE; 
    }
    else if (req.tmsdb.id == 0 && req.tmsdb.type.empty()==true  && req.tmsdb.place !=0)
    {
      mode = MODE_PLACE; 
    }
    else if (req.tmsdb.id == 0 && req.tmsdb.type.empty()==false && req.tmsdb.place !=0)
    {
      mode = MODE_PLACE_TYPE; 
    }
    else if (req.tmsdb.id != 0 && req.tmsdb.type.empty()==true && req.tmsdb.place == sid)
    {
      mode = MODE_HIERARCHY; 
    }
    else if (req.tmsdb.id > 0 && req.tmsdb.id < 1000 || req.tmsdb.id > 20002)
    {
      mode = MODE_ERROR;
    }
    else
    {
      mode = MODE_ERROR;
    }

    //--------------------------------------------------------------------------
    if(is_debug) ROS_INFO("Request mode: %d\n", mode);

    if(mode == MODE_ERROR)
    {
      temp_dbdata.note = "Wrong request! Try to check the command!";
      res.tmsdb.push_back(temp_dbdata);
      return true; 
    }

    //--------------------------------------------------------------------------
    if(mode == MODE_NAME || mode == MODE_NAME_SENSOR)
    {
      // Search the ID, type, etc infomation in ID table
      sprintf(select_query, "SELECT type,id,place FROM rostmsdb.id WHERE name=\"%s\";", req.tmsdb.name.c_str());
      if(is_debug) ROS_INFO("%s\n", select_query);
      mysql_query(connector, select_query);
      result  = mysql_use_result(connector);
      row     = mysql_fetch_row(result);
      if(is_debug) ROS_INFO("%s,%s\n",row[0],row[1]);
      temp_type  = row[0];
      temp_id    = atoi(row[1]);
      temp_place = atoi(row[2]);
      mysql_free_result(result);
    }

    //--------------------------------------------------------------------------
    if(mode == MODE_ID || mode == MODE_ID_SENSOR || mode == MODE_HIERARCHY)
    {
      // Search the type, name, etc infomation in ID table
      sprintf(select_query, "SELECT type,name,place FROM rostmsdb.id WHERE id=%d", req.tmsdb.id);
      if(is_debug) ROS_INFO("%s\n", select_query);
      mysql_query(connector, select_query);
      result  = mysql_use_result(connector);
      row     = mysql_fetch_row(result);
      if(is_debug) ROS_INFO("%s,%s\n",row[0],row[1]);
      temp_type  = row[0];
      temp_name  = row[1];
      temp_place = atoi(row[2]);
      mysql_free_result(result);
    }

    //--------------------------------------------------------------------------
    if (mode == MODE_PLACE)
    {
      for (uint32_t i = 0; i < 7; i++)
      {
        sprintf(select_query, "SELECT * FROM rostmsdb.data_%s WHERE place=%d;", Target[i].c_str(), req.tmsdb.place);
        if(is_debug) ROS_INFO("%s\n", select_query);

        mysql_query(connector, select_query);
        result = mysql_use_result(connector);

        if (result == NULL)
        {
          temp_dbdata.note = "Wrong request! Try to check the target ID or type";
          res.tmsdb.push_back(temp_dbdata);
          return true; 
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
          res.tmsdb.push_back(temp_dbdata);
        }

        if(mysql_num_rows(result) != 0)
          mysql_free_result(result);
      }

      return true;   
    }

    //--------------------------------------------------------------------------
    if(mode == MODE_HIERARCHY)
    {
      bool loop_end_tag = false;
      temp_place = req.tmsdb.id;

      while(temp_place != 0)
      {
        sprintf(select_query, "SELECT * FROM rostmsdb.data_%s WHERE id=%d ORDER BY time DESC LIMIT 1;",
          temp_type.c_str(), 
          temp_place);

        if(is_debug) ROS_INFO("%s\n", select_query);

        mysql_query(connector, select_query);
        result = mysql_use_result(connector);

        if (result == NULL)
        {
          temp_dbdata.note = "Wrong request! Try to check the target ID, place info";
          res.tmsdb.push_back(temp_dbdata);
          return true; 
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
          res.tmsdb.push_back(temp_dbdata);
        }

        mysql_free_result(result);

        if(loop_end_tag)  return true;

        temp_id = temp_dbdata.place; 
        sprintf(select_query, "SELECT type,name,place FROM rostmsdb.id WHERE id=%d", temp_id);
        if(is_debug) ROS_INFO("%s\n", select_query);
        mysql_query(connector, select_query);
        result  = mysql_use_result(connector);
        row     = mysql_fetch_row(result);
        if(is_debug) ROS_INFO("%s,%s,%s\n",row[0],row[1],row[2]);
        temp_type  = row[0];
        temp_name  = row[1];
        temp_place = atoi(row[2]);
        mysql_free_result(result);

        if(temp_id == temp_place)
        {
          loop_end_tag = true;
        }
        else
        {
          loop_end_tag = false;
        }
        temp_place = temp_id;
      }
      return true;
    }

    //--------------------------------------------------------------------------
    if(mode == MODE_ALL)
    {
      sprintf(select_query, "SELECT * FROM rostmsdb.id;");
    }
    if(mode == MODE_TAG_IDTABLE)
    {
      sprintf(select_query, "SELECT * FROM rostmsdb.id WHERE tag LIKE \"%%%s%%\";", req.tmsdb.tag.c_str());
    }
    else if (mode == MODE_NAME_IDTABLE)
    {
      sprintf(select_query, "SELECT * FROM rostmsdb.id WHERE name=\"%s\";", req.tmsdb.name.c_str());
    }
    else if (mode == MODE_NAME)
    {
      sprintf(select_query, "SELECT * FROM rostmsdb.data_%s WHERE name=\"%s\";", temp_type.c_str(), req.tmsdb.name.c_str());
    }
    else if (mode == MODE_NAME_SENSOR)
    {
      sprintf(select_query, "SELECT * FROM rostmsdb.data_%s WHERE name=\"%s\" AND sensor=%d;", temp_type.c_str(), req.tmsdb.name.c_str(), req.tmsdb.sensor);  
    }
    else if (mode == MODE_ID_IDTABLE)
    {
      sprintf(select_query, "SELECT * FROM rostmsdb.id WHERE id=%d;", req.tmsdb.id);
    }
    else if (mode == MODE_ID)
    {
      sprintf(select_query, "SELECT * FROM rostmsdb.data_%s WHERE id=%d;", temp_type.c_str(), req.tmsdb.id);
    }
    else if (mode == MODE_ID_SENSOR)
    {
      sprintf(select_query, "SELECT * FROM rostmsdb.data_%s WHERE id=%d AND sensor=%d;", temp_type.c_str(), req.tmsdb.id, req.tmsdb.sensor);  
    }
    else if (mode == MODE_TYPE_IDTABLE)
    {
      sprintf(select_query, "SELECT * FROM rostmsdb.id WHERE type=\"%s\";", req.tmsdb.type.c_str());
    }
    else if (mode == MODE_TYPE)
    {
      sprintf(select_query, "SELECT * FROM rostmsdb.data_%s;", req.tmsdb.type.c_str());        
    }
    else if (mode == MODE_TYPE_SENSOR)
    {
      sprintf(select_query, "SELECT * FROM rostmsdb.data_%s WHERE sensor=%d;", req.tmsdb.type.c_str(),req.tmsdb.sensor);        
    }
    else if (mode == MODE_PLACE_IDTABLE)
    {
      sprintf(select_query, "SELECT * FROM rostmsdb.id WHERE place=%d;", req.tmsdb.place);        
    }
    else if (mode == MODE_PLACE_TYPE)
    {
      sprintf(select_query, "SELECT * FROM rostmsdb.data_%s WHERE place=%d;", req.tmsdb.type.c_str(), req.tmsdb.place);        
    }

    if(is_debug) ROS_INFO("%s\n", select_query);

    mysql_query(connector, select_query);
    result = mysql_use_result(connector);

    if (result == NULL)
    {
      temp_dbdata.note = "Wrong request! Try to check the target ID, type, name";
      res.tmsdb.push_back(temp_dbdata);
      return true; 
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
      res.tmsdb.push_back(temp_dbdata);
    }

    if(mysql_num_rows(result) != 0)
      mysql_free_result(result);

    return true; 
  }
};

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //Init ROS node
  ros::init(argc, argv, "tms_db_reader");
  DbReader dr;
  ros::spin();
  return 0;
}

//------------------------------------------------------------------------------
//EOF
