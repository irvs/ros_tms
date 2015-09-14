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
