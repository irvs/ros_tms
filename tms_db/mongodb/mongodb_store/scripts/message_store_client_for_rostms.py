#!/usr/bin/env python

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
from tms_msg_db.msg import TmsdbStamped, Tmsdb
import StringIO

class Listener():
    def __init__(self):
        rospy.init_node("message_store_client")
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber("tms_db_publisher/db_publisher", TmsdbStamped, self.callback)

    def callback(self, msg):
        # rospy.loginfo("call message:\n%s", msg)
        self.ros_tms_db = MessageStoreProxy()

        for tmsdb in msg.tmsdb:
            try:
                print self.ros_tms_db.insert_named(tmsdb.name, tmsdb)
            except rospy.ServiceException, e:
                print "ServiceException: %s"%e

    def shutdown(self):
        rospy.loginfo("Stopping the node")

if __name__ == '__main__':
    try:
        Listener()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("message_store_client node terminated.")
