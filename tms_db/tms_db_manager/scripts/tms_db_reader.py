#!/usr/bin/env python
import rospy
import genpy
import pymongo # https://api.mongodb.org/python/2.6.3/
import json
import copy
from bson import json_util
from bson.objectid import ObjectId
from datetime import *
from tms_msg_db.msg import TmsdbStamped, Tmsdb
from tms_msg_db.srv import *
import tms_db_manager.tms_db_util as db_util

client = pymongo.MongoClient('localhost:27017')
db = client.rostmsdb

class TmsDbReader():
    def __init__(self):
        rospy.init_node("tms_db_reader")
        rospy.on_shutdown(self.shutdown)

        db_host = 'localhost'
        db_port = 27017
        self.is_connected = db_util.check_connection(db_host, db_port);
        if not self.is_connected:
            raise Exception("Problem of connection")

        self.collection_list=['data_person','data_robot','data_sensor','data_structure','data_space','data_furniture','data_object']

        self.db_reader_srv = rospy.Service('tms_db_reader', TmsdbGetData, self.dbReaderSrvCallback)

    def dbReaderSrvCallback(self, req):
        rospy.loginfo("Received the service call!")
        rospy.loginfo(req)
        temp_dbdata = Tmsdb()
        result = False

        mode = 0;
        temp_id = 0;
        temp_type = "";
        temp_name = "";
        temp_place = 0;
        sid = 100000

        MODE_ALL           =  0
        MODE_NAME_IDTABLE  =  1
        MODE_NAME          =  2
        MODE_NAME_SENSOR   =  3
        MODE_ID_IDTABLE    =  4
        MODE_ID            =  5
        MODE_ID_SENSOR     =  6
        MODE_TYPE_IDTABLE  =  7
        MODE_TYPE          =  8
        MODE_TYPE_SENSOR   =  9
        MODE_PLACE_IDTABLE = 10
        MODE_PLACE         = 11
        MODE_PLACE_TYPE    = 12
        MODE_HIERARCHY     = 13
        MODE_TAG_IDTABLE   = 14
        MODE_ERROR         = 999

        if req.tmsdb.tag != '':
            mode = MODE_TAG_IDTABLE
        elif (req.tmsdb.id == 0) and (req.tmsdb.type == '') and (req.tmsdb.sensor == 0) and (req.tmsdb.place == 0) and (req.tmsdb.name == ''):
            mode = MODE_ALL
        elif (req.tmsdb.id == sid) and (req.tmsdb.name != ''):
            mode = MODE_NAME_IDTABLE
        elif (req.tmsdb.id == 0) and (req.tmsdb.name != '') and (req.tmsdb.id != sid) and (req.tmsdb.type == '') and (req.tmsdb.sensor == 0) and (req.tmsdb.place == 0):
            mode = MODE_NAME
        elif (req.tmsdb.id == 0) and (req.tmsdb.name != '') and (req.tmsdb.id != sid) and (req.tmsdb.type == '') and (req.tmsdb.sensor != 0) and (req.tmsdb.place == 0):
            mode = MODE_NAME_SENSOR
        elif (req.tmsdb.id > sid) and (req.tmsdb.type == ''):
            req.tmsdb.id -= sid
            mode = MODE_ID_IDTABLE
        elif (req.tmsdb.id != 0) and (req.tmsdb.id != sid) and (req.tmsdb.type == '') and (req.tmsdb.sensor == 0) and (req.tmsdb.place == 0):
            mode = MODE_ID
        elif (req.tmsdb.id != 0) and (req.tmsdb.id != sid) and (req.tmsdb.type = '') and (req.tmsdb.sensor != 0) and (req.tmsdb.place == 0):
            mode = MODE_ID_SENSOR
        elif (req.tmsdb.id == sid) and (req.tmsdb.type != ''):
            mode = MODE_TYPE_IDTABLE
        elif (req.tmsdb.id == 0) and (req.tmsdb.type != '') and (req.tmsdb.sensor == 0) and (req.tmsdb.place ==0):
            mode = MODE_TYPE
        elif (req.tmsdb.id == 0) and (req.tmsdb.type != '') and (req.tmsdb.sensor != 0) and (req.tmsdb.place ==0):
            mode = MODE_TYPE_SENSOR
        elif (req.tmsdb.id == sid) and (req.tmsdb.type == '') and (req.tmsdb.place !=0):
            mode = MODE_PLACE_IDTABLE
        elif (req.tmsdb.id == 0) and (req.tmsdb.type == '') and (req.tmsdb.place !=0):
          mode = MODE_PLACE
        elif (req.tmsdb.id == 0) and (req.tmsdb.type != '') and (req.tmsdb.place !=0):
            mode = MODE_PLACE_TYPE
        elif (req.tmsdb.id != 0) and (req.tmsdb.type == '') and (req.tmsdb.place == sid):
            mode = MODE_HIERARCHY
        elif (req.tmsdb.id > 0) and ((req.tmsdb.id < 1000) or (req.tmsdb.id > 20002)):
            mode = MODE_ERROR
        else:
          mode = MODE_ERROR

        if mode == MODE_ERROR:
            temp_dbdata.note = "Wrong request! Try to check the command!";
            ret = TmsdbGetDataResponse()
            ret.tmsdb.append(temp_dbdata)
            return ret

        #  Search the ID, type, etc infomation in ID table
        if (mode == MODE_NAME) or (mode == MODE_NAME_SENSOR):
            cursor = db['default_data'].find({'name':req.tmsdb.name})
            if cursor[0]['type'] != '':
                collection_name = "data_" + cursor[0]['type']
                # print(collection_name)
                cursor = db[collection_name].find({'name':req.tmsdb.name})
                for doc in cursor:
                    del doc['_id']
                    temp_dbdata = db_util.document_to_msg(doc, Tmsdb)
                    # print(doc)

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

        try:
            if req.tmsdb.name != "":
                cursor = db['default_data'].find({'name':req.tmsdb.name})
                if cursor[0]['type'] != '':
                    collection_name = "data_" + cursor[0]['type']
                    # print(collection_name)
                    cursor = db[collection_name].find({'name':req.tmsdb.name})
                    for doc in cursor:
                        del doc['_id']
                        temp_dbdata = db_util.document_to_msg(doc, Tmsdb)
                        # print(doc)
                    result = True
                else:
                    result = False
            elif req.tmsdb.id != 0:
                print(req.tmsdb.id)
                if (req.tmsdb.id > 2000) and (req.tmsdb.id < 3000):
                    target_id = req.tmsdb.id
                    cursor = db['data_robot'].find({'id':target_id, 'sensor':3005})
                    for doc in cursor:
                        del doc['_id']
                        temp_dbdata = db_util.document_to_msg(doc, Tmsdb)
                        print(doc)
                    result = True
                else:
                    target_id = req.tmsdb.id - 100000
                    cursor = db['default_data'].find({'id':target_id})
                    for doc in cursor:
                        del doc['_id']
                        temp_dbdata = db_util.document_to_msg(doc, Tmsdb)
                        print(doc)
                    result = True
            else:
                result = False
        except:
            result = False

        ret = TmsdbGetDataResponse()

        if result == False:
            temp_dbdata.note = "Wrong request! Try to check the target type and name"

        ret.tmsdb.append(temp_dbdata)
        return ret

    def shutdown(self):
        rospy.loginfo("Stopping the node")

if __name__ == '__main__':
    try:
        TmsDbReader()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("tms_db_reader node terminated.")
