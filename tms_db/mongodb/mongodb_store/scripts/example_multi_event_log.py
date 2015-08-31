#!/usr/bin/env python

import rospy
from mongodb_store_msgs.msg import StringPairList, StringPair
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool
import StringIO
from datetime import *

if __name__ == '__main__':
    rospy.init_node("example_multi_event_log")

    try:

        # let's say we have a couple of things that we need to store together
        # these could be some sensor data, results of processing etc.
        pose = Pose(Point(0, 1, 2), Quaternion(3, 4,  5, 6))
        point = Point(7, 8, 9)
        quaternion = Quaternion(10, 11, 12, 13)
        # note that everything that is pass to the message_store must be a ros message type
        #therefore use std_msg types for standard data types like float, int, bool, string etc
        result = Bool(True)


        # we will store our results in a separate collection
        msg_store = MessageStoreProxy(collection='pose_results')
        # save the ids from each addition
        stored = []
        stored.append([pose._type, msg_store.insert(pose)])
        stored.append([point._type, msg_store.insert(point)])
        stored.append([quaternion._type, msg_store.insert(quaternion)])
        stored.append([result._type, msg_store.insert(result)])
        
        # now store ids togther in store, addition types for safety
        spl = StringPairList()
        for pair in stored:
            spl.pairs.append(StringPair(pair[0], pair[1]))

        # and add some meta information
        meta = {}
        meta['description'] = "this wasn't great"    
        meta['result_time'] = datetime.utcfromtimestamp(rospy.get_rostime().to_sec())
        msg_store.insert(spl, meta = meta)

        # now let's get all our logged data back
        results = msg_store.query(StringPairList._type)
        for message, meta in results:
            if 'description' in meta:
                print 'description: %s' % meta['description']
            print 'result time (UTC from rostime): %s' % meta['result_time']            
            print 'inserted at (UTC from rostime): %s' % meta['inserted_at']
            pose = msg_store.query_id(message.pairs[0].second, Pose._type)
            point = msg_store.query_id(message.pairs[1].second, Point._type)
            quaternion = msg_store.query_id(message.pairs[2].second, Quaternion._type)
            result = msg_store.query_id(message.pairs[3].second, Bool._type)

        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


        
