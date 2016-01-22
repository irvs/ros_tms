#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import threading
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

class TmsDbReplayer():
    def __init__(self):
        rospy.init_node("tms_db_replayer")
        rospy.on_shutdown(self.shutdown)

        db_host = 'localhost'
        db_port = 27017
        self.is_connected = db_util.check_connection(db_host, db_port)
        if not self.is_connected:
            raise Exception("Problem of connection")

        self.thread_list = []
        result = rospy.Service('tms_db_replayer', TmsdbCallReplayer, self.handleCallReplayer)

        rospy.spin()

    def handleCallReplayer(self, req):
        rospy.loginfo('Received calling')

        self.request_id = req.request_id
        self.topic_name = req.topic_name
        self.start_date = req.start_date
        self.end_date = req.end_date
        self.play_speed = req.play_speed

        thread = threading.Thread(target=self.sendDbHistoryInformation)
        self.thread_list.append(thread)
        thread.daemon = True
        thread.start()

        return TmsdbCallReplayerResponse(0)

    def sendDbHistoryInformation(self):
        # Input
        d1 = datetime.strptime(self.start_date, '%Y-%m-%dT%H:%M:%S')
        d2 = datetime.strptime(self.end_date, '%Y-%m-%dT%H:%M:%S')
        t1 = 'ISODate('+d1.isoformat()+')'
        t2 = 'ISODate('+d2.isoformat()+')'

        self.rate = rospy.Rate(100)  # [Hz] : Keep consistence with db_publisher
        self.data_pub = rospy.Publisher(self.topic_name, TmsdbStamped, queue_size=10)

        # Extract data from DB
        temp_dbdata = Tmsdb()
        object_information = TmsdbStamped()
        rospy.loginfo('Extracting data: from {0} to {1}'.format(t1,t2))
        db.history.create_index([('time', pymongo.ASCENDING), ('id', pymongo.ASCENDING)])

        # Publish data
        rospy.loginfo('Start to publish data.')
        prev_time = d1
        t_delta = timedelta(milliseconds=10*self.play_speed)
        curr_time = d1 + t_delta
        while rospy.is_shutdown() or curr_time <= d2:
            t1 = 'ISODate('+prev_time.isoformat()+')'
            t2 = 'ISODate('+curr_time.isoformat()+')'
            rospy.loginfo('from {0} to {1}'.format(t1,t2))

            cursor = db.history.find({'$or':[json.loads('{"id":'+str(x)+'}') for x in self.request_id], 'time':{'$gte':t1,'$lt':t2}, 'state':1})
            for doc in cursor:
                del doc['_id']
                temp_dbdata = db_util.document_to_msg(doc, Tmsdb)
                object_information.tmsdb.append(temp_dbdata)
            self.data_pub.publish(object_information)
            prev_time = curr_time
            curr_time += t_delta
            self.rate.sleep()

        rospy.loginfo('Finish publishing data.')

    def shutdown(self):
        rospy.loginfo("Stopping the node")

if __name__ == '__main__':
    try:
        TmsDbReplayer()
    except rospy.ROSInterruptException:
        rospy.loginfo("tms_db_replayer node terminated.")
