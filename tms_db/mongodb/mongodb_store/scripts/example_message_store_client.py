#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
import StringIO


if __name__ == '__main__':
    rospy.init_node("example_message_store_client")


    msg_store = MessageStoreProxy()

    p = Pose(Point(0, 1, 2), Quaternion(3, 4,  5, 6))
  
    try:


        # insert a pose object with a name, store the id from db
        p_id = msg_store.insert_named("my favourite pose", p)
 
        # you don't need a name (note that this p_id is different than one above)
        p_id = msg_store.insert(p)

        p_id = msg_store.insert(['test1', 'test2'])         

        # get it back with a name
        print msg_store.query_named("my favourite pose", Pose._type)

        p.position.x = 666

        # update it with a name
        msg_store.update_named("my favourite pose", p)

        p.position.y = 2020

        # update the other inserted one using the id
        msg_store.update_id(p_id, p)

        stored_p, meta = msg_store.query_id(p_id, Pose._type)

        assert stored_p.position.x == 666
        assert stored_p.position.y == 2020
        print "stored object ok"
        print "stored object inserted at %s (UTC rostime) by %s" % (meta['inserted_at'], meta['inserted_by'])
        print "stored object last updated at %s (UTC rostime) by %s" % (meta['last_updated_at'], meta['last_updated_by'])

        # some other things you can do...

        # get it back with a name
        print msg_store.query_named("my favourite pose", Pose._type)


        # try to get it back with an incorrect name, so get None instead
        print msg_store.query_named("my favourite position", Pose._type)

        # get all poses  
        print msg_store.query(Pose._type)

        # get the latest one pose
        print msg_store.query(Pose._type, sort_query=[("$natural", -1)], single=True)

        # get all non-existant typed objects, so get an empty list back
        print msg_store.query( "not my type")
        
        # get all poses where the y position is 1
        print msg_store.query(Pose._type, {"position.y": 1})

        # get all poses where the y position greater than 0
        print msg_store.query(Pose._type, {"position.y": {"$gt": 0}})

        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


        
