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
import tms_db_manager.tms_db_util.py as db_util

client = pymongo.MongoClient('localhost:27017')
db = client.rostmsdb

class TmsDbManager():
    def __init__(self):
        rospy.init_node("tms_db_manager")
        rospy.on_shutdown(self.shutdown)
        db_host = 'localhost'
        db_port = 27017
        self.is_connected = db_util.check_connection(db_host, db_port);
        if not self.is_connected:
            raise Exception("Problem of connection")

        rospy.Timer(rospy.Duration(2), self.manageDataCallback) #60sec
        rospy.Timer(rospy.Duration(12*60*60), self.removeForeverDataCallback) # 12hour

    def manageDataCallback(self):
        print(datetime.utcfromtimestamp(rospy.get_rostime().to_sec()))

    def removeForeverDataCallback(self):
        print(datetime.utcfromtimestamp(rospy.get_rostime().to_sec()))

    def shutdown(self):
        rospy.loginfo("Stopping the node")

if __name__ == '__main__':
    try:
        TmsDbManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("tms_db_manager node terminated.")
