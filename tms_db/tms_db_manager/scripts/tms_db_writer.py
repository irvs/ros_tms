#!/usr/bin/env python
import rospy
import genpy
import pymongo # v2.6.3
import json
import copy
from bson import json_util
from bson.objectid import ObjectId
from datetime import *
from tms_msg_db.msg import TmsdbStamped, Tmsdb
import tms_db_manager.tms_db_util as db_util

client = pymongo.MongoClient('localhost:27017')
db = client.rostmsdb

class TmsDbWriter():
    def __init__(self):
        rospy.init_node("tms_db_writer_")
        rospy.on_shutdown(self.shutdown)
        db_host = 'localhost'
        db_port = 27017
        self.is_connected = db_util.check_connection(db_host, db_port);
        if not self.is_connected:
            raise Exception("Problem of connection")
        rospy.Subscriber("tms_db_data", TmsdbStamped, self.dbWriteCallback)
        self.writeInitData();

    def dbWriteCallback(self, msg):
        # rospy.loginfo("writing the one msg")
        for tmsdb in msg.tmsdb:
            try:
                doc = db_util.msg_to_document(tmsdb)
                # store into db of history_data
                # rospy.loginfo("store into db of history_data")
                db.history_data.insert(doc)
                # store into db of data_xxx
                category="data_" + doc['type']
                # print(category)
                # print(doc['name'])
                # rospy.loginfo("update into db of " + category)
                result = db[category].find({"name": doc['name']})
                # print(result.count())
                if result.count() >= 1:
                    del doc['_id']
                result = db[category].update(
                    {"name": doc['name']},
                    doc,
                    upsert=True
                )
                # print(result)
            except rospy.ServiceException, e:
                print "ServiceException: %s"%e

    def writeInitData(self):
        cursor = db.default_data.find({"type": "furniture"})
        for doc in cursor:
            # print(doc['name'])
            result = db['data_furniture'].update(
                {"name": doc['name']},
                doc,
                upsert=True
            )
            # print(result)
        cursor = db.default_data.find({"type": "robot"})
        for doc in cursor:
            # print(doc['name'])
            result = db['data_robot'].update(
                {"name": doc['name']},
                doc,
                upsert=True
            )
            # print(result)
        rospy.loginfo("Writed the init data using db of default_data.")

    def shutdown(self):
        rospy.loginfo("Stopping the node")

if __name__ == '__main__':
    try:
        TmsDbWriter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("tms_db_writer node terminated.")

# test topic message
# rostopic pub /tms_db_data tms_msg_db/TmsdbStamped "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: ''
# tmsdb:
# - {time: '', type: '', id: 0, name: '', x: 0.0, y: 0.0, z: 0.0, rr: 0.0, rp: 0.0,
#   ry: 0.0, offset_x: 0.0, offset_y: 0.0, offset_z: 0.0, joint: '', weight: 0.0, rfid: '',
#   etcdata: '', place: 0, extfile: '', sensor: 0, probability: 0.0, state: 0, task: '',
#   note: '', tag: ''}"
