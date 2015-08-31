#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Provides a service to store ROS message objects in a mongodb database in JSON.
"""

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
import pymongo
import json
from bson import json_util
from mongodb_store_msgs.msg import  StringPair, StringPairList
from bson.objectid import ObjectId
from datetime import *

MongoClient = dc_util.import_MongoClient()

class MessageStore(object):
    def __init__(self, replicate_on_write=False):

        self.replicate_on_write = replicate_on_write

        use_daemon = rospy.get_param('mongodb_use_daemon', False)
        db_host = rospy.get_param('mongodb_host')
        db_port = rospy.get_param('mongodb_port')
        if use_daemon:
            is_daemon_alive = dc_util.check_connection_to_mongod(db_host, db_port)
            if not is_daemon_alive:
                raise Exception("No Daemon?")
        else:
            have_dc = dc_util.wait_for_mongo()
            if not have_dc:
                raise Exception("No Datacentre?")

        self._mongo_client=MongoClient(db_host, db_port)


        extras = rospy.get_param('mongodb_store_extras', [])
        self.extra_clients = []
        for extra in extras:
            try:
                self.extra_clients.append(MongoClient(extra[0], extra[1]))
            except pymongo.errors.ConnectionFailure, e:
                rospy.logwarn('Could not connect to extra datacentre at %s:%s' % (extra[0], extra[1]))

        if self.replicate_on_write:            
            rospy.loginfo('Replicating content to a futher %s datacentres',len(self.extra_clients))
        else:
            rospy.loginfo('Querying content in a futher %s datacentres',len(self.extra_clients))

        # advertise ros services
        for attr in dir(self):
            if attr.endswith("_ros_srv"):
                service=getattr(self, attr)
                rospy.Service("/message_store/"+attr[:-8], service.type, service)



    def insert_ros_srv(self, req):
        """
        Receives a 
        """


        # deserialize data into object
        obj = dc_util.deserialise_message(req.message)        
        # convert input tuple to dict
        meta = dc_util.string_pair_list_to_dictionary(req.meta)
        # get requested collection from the db, creating if necessary
        collection = self._mongo_client[req.database][req.collection]

        # try:
        meta['inserted_at'] = datetime.utcfromtimestamp(rospy.get_rostime().to_sec())
        meta['inserted_by'] = req._connection_header['callerid']
        obj_id = dc_util.store_message(collection, obj, meta)


        if self.replicate_on_write:            
            # also do insert to extra datacentres, making sure object ids are consistent
            for extra_client in self.extra_clients:
                extra_collection = extra_client[req.database][req.collection]            
                dc_util.store_message(extra_collection, obj, meta, obj_id)

        return str(obj_id)   
        # except Exception, e:
            # print e    
             
    insert_ros_srv.type=dc_srv.MongoInsertMsg
             
    def delete_ros_srv(self, req):
        """
        Deletes a message by ID 
        """
        # Get the message
        collection = self._mongo_client[req.database][req.collection]
        docs = dc_util.query_message(collection, {"_id": ObjectId(req.document_id)}, find_one=True)
        if len(docs) != 1:
            return False

        message = docs[0]
        
        # Remove the doc
        collection.remove({"_id": ObjectId(req.document_id)})
        
        # But keep it into "trash"
        bk_collection = self._mongo_client[req.database][req.collection + "_Trash"]
        bk_collection.save(message)


        # also repeat in extras
        for extra_client in self.extra_clients:
            extra_collection = extra_client[req.database][req.collection]            
            extra_collection.remove({"_id": ObjectId(req.document_id)})
            extra_bk_collection = extra_client[req.database][req.collection + "_Trash"]
            extra_bk_collection.save(message)


        return True        
    delete_ros_srv.type=dc_srv.MongoDeleteMsg
             

    def update_ros_srv(self, req):
        """
        Updates a msg in the store
        """
        # rospy.lrosoginfo("called")
        collection = self._mongo_client[req.database][req.collection]

        # build the query doc         
        obj_query = self.to_query_dict(req.message_query, req.meta_query)

        # restrict results to have the type asked for
        obj_query["_meta.stored_type"] = req.message.type

        # TODO start using some string constants!

        rospy.logdebug("update spec document: %s", obj_query) 

        # deserialize data into object
        obj = dc_util.deserialise_message(req.message)        
      
        meta = dc_util.string_pair_list_to_dictionary(req.meta)
        meta['last_updated_at'] = datetime.utcfromtimestamp(rospy.get_rostime().to_sec())
        meta['last_updated_by'] = req._connection_header['callerid']
      
        (obj_id, altered) = dc_util.update_message(collection, obj_query, obj, meta, req.upsert)

        if self.replicate_on_write:            
            # also do update to extra datacentres
            for extra_client in self.extra_clients:
                extra_collection = extra_client[req.database][req.collection]            
                dc_util.update_message(extra_collection, obj_query, obj, meta, req.upsert)

        return str(obj_id), altered
    update_ros_srv.type=dc_srv.MongoUpdateMsg
       

    def to_query_dict(self, message_query, meta_query):            
        """
        Decodes and combines the given StringPairList queries into a single mongodb query
        """
        obj_query = dc_util.string_pair_list_to_dictionary(message_query)        
        bare_meta_query = dc_util.string_pair_list_to_dictionary(meta_query)
        for (k, v) in bare_meta_query.iteritems():
            obj_query["_meta." + k] = v

        return obj_query


    def query_messages_ros_srv(self, req):
        """
        Returns t
        """
        collection = self._mongo_client[req.database][req.collection]

        # build the query doc         
        obj_query = self.to_query_dict(req.message_query, req.meta_query)

        # restrict results to have the type asked for
        obj_query["_meta.stored_type"] = req.type

        # TODO start using some string constants!

        rospy.logdebug("query document: %s", obj_query) 
        
        # this is a list of entries in dict format including meta
        sort_query_dict = dc_util.string_pair_list_to_dictionary(req.sort_query)
        sort_query_tuples = []
        for k,v in sort_query_dict.iteritems():
            try:
                sort_query_tuples.append((k, int(v)))
            except ValueError:
                sort_query_tuples.append((k,v))

        entries =  dc_util.query_message(collection, obj_query, sort_query_tuples, req.single)

        # keep trying clients until we find an answer
        for extra_client in self.extra_clients:
            if len(entries) == 0:
                extra_collection = extra_client[req.database][req.collection]            
                entries =  dc_util.query_message(extra_collection, obj_query, sort_query_tuples, req.single)
                if len(entries) > 0:
                    rospy.loginfo("found result in extra datacentre")             
            else:
                break


        # rospy.logdebug("entries: %s", entries) 

        serialised_messages = ()
        metas = ()

        for entry in entries:
            # load the class object for this type
            # TODO this should be the same for every item in the list, so could reuse
            cls = dc_util.load_class(entry["_meta"]["stored_class"])
            # instantiate the ROS message object from the dictionary retrieved from the db
            message = dc_util.dictionary_to_message(entry, cls)            
            # the serialise this object in order to be sent in a generic form
            serialised_messages = serialised_messages + (dc_util.serialise_message(message), )            
            # add ObjectID into meta as it might be useful later
            entry["_meta"]["_id"] = entry["_id"]
            # serialise meta
            metas = metas + (StringPairList([StringPair(dc_srv.MongoQueryMsgRequest.JSON_QUERY, json.dumps(entry["_meta"], default=json_util.default))]), )

        return [serialised_messages, metas]
        
    query_messages_ros_srv.type=dc_srv.MongoQueryMsg


if __name__ == '__main__':
    rospy.init_node("message_store")

    store = MessageStore()
    
    rospy.spin()
