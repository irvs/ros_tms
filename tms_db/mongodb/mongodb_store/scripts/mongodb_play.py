#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store.util as mg_util
import sys
import time
import pymongo
from multiprocessing import Process
import calendar
import datetime
import threading
import multiprocessing 
from rosgraph_msgs.msg import Clock
import signal
import Queue
from optparse import OptionParser

MongoClient = mg_util.import_MongoClient()

TIME_KEY = '_meta.inserted_at'

def max_time(collection):
    return collection.find_one(sort=[(TIME_KEY, pymongo.DESCENDING)])['_meta']['inserted_at']

def min_time(collection):
    return collection.find_one(sort=[(TIME_KEY, pymongo.ASCENDING)])['_meta']['inserted_at']

def to_ros_time(dt):
    return rospy.Time(calendar.timegm(dt.utctimetuple()), dt.microsecond * 1000)

def to_datetime(rt):
    return datetime.datetime.utcfromtimestamp(rt.secs) + datetime.timedelta(microseconds     = rt.nsecs / 1000)

def ros_time_strftime(rt, format):
    """ converts a ros time to a datetime and calls strftime on it with the given format """
    return to_datetime(rt).strftime(format)


class PlayerProcess(object):
    
    def __init__(self, event, start_time, end_time):
        super(PlayerProcess, self).__init__()

        self.event = event

        self.start_time = start_time
        self.end_time = end_time

        self.running = multiprocessing.Value('b', True)
        self.player_process = multiprocessing.Process(target=self.run, args=[self.running])
        
    def start(self):        
        self.player_process.start()

    def stop(self):        
        self.running.value = False

    def join(self):        
        self.player_process.join()

    def is_running(self):
        return self.running.value

class TopicPlayer(PlayerProcess):
    """ """
    def __init__(self, mongodb_host, mongodb_port, db_name, collection_name, event, start_time, end_time):
        super(TopicPlayer, self).__init__(event, start_time, end_time)
        
        self.mongodb_host = mongodb_host
        self.mongodb_port = mongodb_port
        self.db_name = db_name
        self.collection_name = collection_name


    def init(self, running):
        """ Called in subprocess to do process-specific initialisation """

        rospy.init_node("mongodb_playback_%s" % self.collection_name) 

        # clear signal handlers in this child process, rospy will handle signals for us
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        self.mongo_client=MongoClient(self.mongodb_host, self.mongodb_port)
        self.collection = self.mongo_client[self.db_name][self.collection_name]

        # two threads running here, the main one does the publishing

        # the second one populates the qeue of things to publish 

        # how many to 
        buffer_size = 50
        self.to_publish = Queue.Queue(maxsize=buffer_size)
        self.queue_thread = threading.Thread(target=self.queue_from_db, args=[running])
        self.queue_thread.start()


    def queue_from_db(self, running):
        # make sure there's an index on time in the collection so the sort operation doesn't require the whole collection to be loaded
        self.collection.ensure_index(TIME_KEY)                        
        # get all documents within the time window, sorted ascending order by time
        documents = self.collection.find({TIME_KEY: { '$gte': to_datetime(self.start_time), '$lte': to_datetime(self.end_time)}}, sort=[(TIME_KEY, pymongo.ASCENDING)])

        if documents.count() == 0:
            rospy.logwarn('No messages to play back from topic %s' % self.collection_name)
            return
        else:
            rospy.logdebug('Playing back %d messages', documents.count())


        # load message class for this collection, they should all be the same
        msg_cls = mg_util.load_class(documents[0]["_meta"]["stored_class"])

        # publisher won't be used until something is on the queue, so it's safe to construct it here
        self.publisher = rospy.Publisher(documents[0]["_meta"]["topic"], msg_cls, latch = documents[0]["_meta"]["latch"], queue_size = 10)

        for document in documents:
            if running.value:
                # instantiate the ROS message object from the dictionary retrieved from the db
                message = mg_util.dictionary_to_message(document, msg_cls)            
                # print (message, document["_meta"]["inserted_at"])
                # put will only work while there is space in the queue, if not it will block until another take is performed
                self.to_publish.put((message, to_ros_time(document["_meta"]["inserted_at"])))            
            else:
                break

        rospy.logdebug('All messages queued for topic %s' % self.collection_name)


    def run(self, running):

        self.init(running)

        # wait until sim clock has initialised
        while rospy.get_rostime().secs == 0:
            # can't use rospy time here as if clock is 0 it will wait forever
            time.sleep(0.2)

        rospy.logdebug('Topic playback ready %s %s' % (self.collection.name, rospy.get_param('use_sim_time')))

        # wait for the signal to start
        self.event.wait()

        timeout = 1

        while running.value:
            try: 
                msg_time_tuple = self.to_publish.get(timeout=timeout)
                publish_time = msg_time_tuple[1]
                msg = msg_time_tuple[0]

                now = rospy.get_rostime() 

                # if we've missed our window
                if publish_time < now:
                    rospy.logwarn('Message out of sync by %f', (now - publish_time).to_sec())                
                else:
                    delay = publish_time - now                    
                    rospy.sleep(delay)
    
                # rospy.loginfo('diff %f' % (publish_time - rospy.get_rostime()).to_sec())
                self.publisher.publish(msg)
        
            except Queue.Empty, e:
                pass
                

        self.queue_thread.join()
        self.mongo_client.disconnect()
        rospy.loginfo('Topic playback finished %s' % self.collection.name)            



class ClockPlayer(PlayerProcess):
    """ Plays a clock message in a separate thread"""
    def __init__(self, event, start_time, end_time, pre_roll = rospy.Duration(0), post_roll = rospy.Duration(0)):
        super(ClockPlayer, self).__init__(event, start_time, end_time)
        self.start_time = start_time
        self.end_time = end_time
        self.pre_roll = pre_roll
        self.post_roll = post_roll        
      
    def init(self):

        # we handle shutdown for this process
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        # make sure this node doesn't use sim time
        rospy.set_param('use_sim_time', False)

        rospy.init_node('mongodb_playback_clock_player')
    
        # switch to simulated time, note that as this is after the init_node, this node DOES NOT use sim time
        rospy.set_param('use_sim_time', True)

        # topic to public clock on
        self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)

        # send the first message to get time off 0
        self.clock_pub.publish(Clock(clock=(self.start_time - self.pre_roll)))

        # notify everyone else that they can move on
        self.event.set()


    def run(self, running):

        self.init()

        start = self.start_time - self.pre_roll
        end = self.end_time + self.post_roll
        
        # start value
        clock_msg = Clock(clock=start)

        # timing details, should be moved to constructor parameters
        updates_hz = 1000.0
        rate = rospy.Rate(updates_hz)
        # this assumes close to real-time playback
        update = rospy.Duration(1.0 / updates_hz)

        # wait for the signal to start
        self.event.wait()

        while running.value and clock_msg.clock <= end:

            # update time
            clock_msg.clock += update

            # publish time
            self.clock_pub.publish(clock_msg)

            rate.sleep()

        rospy.logdebug('Playback clock finished')
        running.value = False
 


class MongoPlayback(object):
    """ Plays back stored topics from the mongodb_store """

    def __init__(self): 
        super(MongoPlayback, self).__init__() 

        self.mongodb_host = rospy.get_param("mongodb_host")
        self.mongodb_port = rospy.get_param("mongodb_port")
        self.mongo_client=MongoClient(self.mongodb_host, self.mongodb_port)
        self.stop_called = False
        

    def setup(self, database_name, req_topics):
        """ Read in details of requested playback collections. """

        if database_name not in self.mongo_client.database_names():
            raise Exception('Unknown database %s' % database_name)

        database = self.mongo_client[database_name] 
        collection_names = database.collection_names(include_system_collections=False) 

        req_topics = set(map(mg_util.topic_name_to_collection_name, req_topics))

        if len(req_topics) > 0:             
            topics = req_topics.intersection(collection_names) 
            dropped = req_topics.difference(topics)
            if(len(dropped) > 0):
                print('WARNING Dropped non-existant requested topics for playback: %s' % dropped)
        else: 
            topics = set(collection_names)

        print('Playing back topics %s' % topics)

        # create mongo collections
        collections = [database[collection_name] for collection_name in topics]

        # make sure they're easily accessible by time
        for collection in collections:
            collection.ensure_index(TIME_KEY)


        # get the min and max time across all collections, conver to ros time
        start_time = to_ros_time(min(map(min_time, collections)))
        end_time =  to_ros_time(max(map(max_time, collections)))

        # we don't need a connection any more
        self.mongo_client.disconnect()

        # rospy.loginfo('Playing back from %s' % to_datetime(start_time))
        # rospy.loginfo('.............. to %s' % to_datetime(end_time))

        self.event = multiprocessing.Event()

        # create clock thread        
        pre_roll = rospy.Duration(2)
        post_roll = rospy.Duration(0)
        self.clock_player = ClockPlayer(self.event, start_time, end_time, pre_roll, post_roll)

        # create playback objects
        self.players = map(lambda c: TopicPlayer(self.mongodb_host, self.mongodb_port, database_name, c, self.event, start_time - pre_roll, end_time + post_roll), topics)       


    def start(self):
        self.clock_player.start()

        # wait until clock has set sim time
        self.event.wait()
        self.event.clear()

        # this creates new processes and publishers for each topic
        for player in self.players:
            player.start()        

        # all players wait for this before starting -- 
        # todo: it could happen that his gets hit before all are constructed though
        self.event.set()

    def join(self):

        self.clock_player.join()

        # if clock runs out but we weren't killed then we need ot stop other processes
        if not self.stop_called:
            self.stop()

        for player in self.players:
            player.join()



    def stop(self):
        self.stop_called = True
        self.clock_player.stop()
        for player in self.players:
            player.stop()


    def is_running(self):
        return self.clock_player.is_running()


def main(argv):

    myargv = rospy.myargv(argv=argv)

    parser = OptionParser()
    parser.usage += " [TOPICs...]"
    parser.add_option("--mongodb-name", dest="mongodb_name",
                      help="Name of DB in which to store values",
                      metavar="NAME", default="roslog")
    (options, args) = parser.parse_args(myargv)

    database_name = options.mongodb_name
    topics = set(args[1:])

    playback = MongoPlayback()

    def signal_handler(signal, frame):
        playback.stop()

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)





    playback.setup(database_name, topics)
    playback.start()    
    playback.join()
    rospy.set_param('use_sim_time', False)
    


# processes load main so move init_node out 
if __name__ == "__main__":
    main(sys.argv)
