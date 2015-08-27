#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64, Bool
from threading import Thread
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy

class TestPublisher(object):

    def __init__(self, topic, delay, rate, count=20):
        super(TestPublisher, self).__init__()
        self.publisher = rospy.Publisher(topic, Int64, queue_size=min(10, count))
        self.count = count
        self.delay = delay
        self.rate = rate
        self.thread = Thread(target=self.publish, name=topic)


    def publish(self):
        rospy.sleep(self.delay)

        for i in range(self.count):
            self.publisher.publish(i)
            self.rate.sleep()




if __name__ == '__main__':
 
    rospy.init_node("mongodb_log_test_publisher")


    # topic, delay, rate, count
    to_publish = [ ('test_0', rospy.Duration(10), rospy.Rate(1), 20), ('test_1', rospy.Duration(20), rospy.Rate(1), 20) ] 

    publishers = map(lambda tup: TestPublisher(*tup), to_publish)
    map(lambda pub: pub.thread.start(), publishers)
    map(lambda pub: pub.thread.join(), publishers)


    # now I should be able to get these back from the datacentre

    for target in to_publish:
        msg_store = MessageStoreProxy(database='roslog', collection=target[0])
        print len(msg_store.query(Int64._type)) == target[3]
        

