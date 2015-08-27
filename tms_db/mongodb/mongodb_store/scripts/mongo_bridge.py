#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Bridges mongo to ros, providing services to query the database.
Not efficient, so should be only used when no other option is available; if
you can use a mongodb client library like pymongo, use it, if you need to query
from client side javascript on a webpage with rosbridge/librosjs then use this.

"""

import rospy
import mongodb_store.util as dc_util
from mongodb_store.srv import *
import pymongo
import bson.json_util 
import json

MongoClient = dc_util.import_MongoClient()

class MongoBridge(object):
    def __init__(self):
        rospy.init_node("mongo_bridge")

        have_dc = dc_util.wait_for_mongo()
        if not have_dc:
            raise Exception("No Datacentre?")

        self._mongo_client=pymongo.MongoClient(rospy.get_param("mongodb_host"),
                                               rospy.get_param("mongodb_port") )

        # advertise ros services
        for attr in dir(self):
            if attr.endswith("_ros_srv"):
                service=getattr(self, attr)
                rospy.Service("/mongo_bridge/"+attr[:-8], service.type, service)


    def find_ros_srv(self, req):
        collection = self._mongo_client[req.db][req.collection]
        res=collection.find(json.loads(req.query, object_hook=json_util.object_hook))
        docs=[i for i in res]
        return json.dumps(docs, default=json_util.default)
    find_ros_srv.type=MongoFind

    def update_ros_srv(self,req):
        collection = self._mongo_client[req.db][req.collection]
        res=collection.update(json.loads(req.query, object_hook=json_util.object_hook),
                              json.loads(req.update, object_hook=json_util.object_hook))
        return json.dumps(docs, default=json_util.default)
    update_ros_srv.type=MongoUpdate
                                               
    def insert_ros_srv(self,req):
        collection = self._mongo_client[req.db][req.collection]
        res=collection.insert(json.loads(req.document, object_hook=json_util.object_hook))
        return json.dumps(res, default=json_util.default)
    insert_ros_srv.type=MongoInsert
                                              

if __name__ == '__main__':
    bridge = MongoBridge()
    
    rospy.spin()
