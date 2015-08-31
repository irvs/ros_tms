#!/usr/bin/env python

import rospy
import subprocess
import sys
import os
import re
import signal
import errno
from std_srvs.srv import *
import shutil

import mongodb_store.util

if not mongodb_store.util.check_for_pymongo():
    sys.exit(1)

MongoClient = mongodb_store.util.import_MongoClient()
    
import pymongo

def is_socket_free(host, port):
    import socket;
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    result = sock.connect_ex((host, port))
    return result != 0

class MongoServer(object):
    def __init__(self):
        rospy.init_node("mongodb_server", anonymous=True)#, disable_signals=True)
      
 
        # Has the db already gone down, before the ros node?
        self._gone_down = False

        self._ready = False # is the db ready: when mongo says "waiting for connection"


        self.test_mode = rospy.get_param("~test_mode", False)
        self.repl_set = rospy.get_param("~repl_set", None)


        if self.test_mode:
            import random
            
            default_host = "localhost"            
            default_port = random.randrange(49152,65535)

            count = 0
            while not is_socket_free(default_host, default_port):
                default_port = random.randrange(49152,65535)
                count += 1
                if count > 100:
                    rospy.logerr("Can't find a free port to run the test server on.")
                    sys.exit(1)                    
            
            self.default_path = "/tmp/ros_mongodb_store_%d" % default_port
            os.mkdir(self.default_path)
        else:
            default_host = "localhost"
            default_port = 27017
            self.default_path = "/opt/ros/mongodb_store"

        # Get the database path
        self._db_path = rospy.get_param("~database_path", self.default_path)
        is_master = rospy.get_param("~master", True)

        if is_master:
            self._mongo_host = rospy.get_param("mongodb_host", default_host)
            rospy.set_param("mongodb_host",self._mongo_host)
            self._mongo_port = rospy.get_param("mongodb_port", default_port)
            rospy.set_param("mongodb_port",self._mongo_port)            
        else:
            self._mongo_host = rospy.get_param("~host")     
            self._mongo_port = rospy.get_param("~port")

        rospy.loginfo("Mongo server address: "+self._mongo_host+":"+str(self._mongo_port))

        # Check that mongodb is installed
        try:
            mongov = subprocess.check_output(["mongod","--version"])
            match = re.search("db version v(\d+\.\d+\.\d+)",mongov)
            self._mongo_version=match.group(1)
        except subprocess.CalledProcessError:
            rospy.logerr("Can't find MongoDB executable. Is it installed?\nInstall it with  \"sudo apt-get install mongodb\"")
            sys.exit(1)
        rospy.loginfo("Found MongoDB version " + self._mongo_version)

        # Check that the provided db path exists.
        if not os.path.exists(self._db_path):
            rospy.logerr("Can't find database at supplied path " + self._db_path + ". If this is a new DB, create it as an empty directory.")
            sys.exit(1)

        # Advertise ros services for db interaction
        self._shutdown_srv = rospy.Service("/datacentre/shutdown", Empty, self._shutdown_srv_cb)
        self._wait_ready_srv = rospy.Service("/datacentre/wait_ready",Empty,self._wait_ready_srv_cb)

        rospy.on_shutdown(self._on_node_shutdown)
        
        # Start the mongodb server
        self._mongo_loop()

        
        
    def _mongo_loop(self):

        # Blocker to prevent Ctrl-C being passed to the mongo server
        def block_mongo_kill():
            os.setpgrp()
#            signal.signal(signal.SIGINT, signal.SIG_IGN)

        cmd = ["mongod","--dbpath",self._db_path,"--port",str(self._mongo_port)]
        if self.repl_set is not None:
            cmd.append("--replSet")
            cmd.append(self.repl_set)
        self._mongo_process = subprocess.Popen(cmd,
                                         stdout=subprocess.PIPE,
                                         preexec_fn = block_mongo_kill)

        while self._mongo_process.poll() is None:# and not rospy.is_shutdown():
            try:
                stdout = self._mongo_process.stdout.readline()
            except IOError, e: # probably interupt because shutdown cut it up
                if e.errno == errno.EINTR:
                    continue
                else:
                    raise
            if stdout is not None:
                if stdout.find("ERROR") !=-1:
                    rospy.logerr(stdout.strip())
                else:
                    rospy.loginfo(stdout.strip())

                if stdout.find("waiting for connections on port") !=-1:
                    self._ready=True
                    if self.repl_set is not None:
                        try:
                            self.initialize_repl_set()
                        except Exception as e:
                            rospy.logwarn("initialzing replSet failed: %s" % e)

        if not rospy.is_shutdown():
            rospy.logerr("MongoDB process stopped!")
            
        if self._mongo_process.returncode!=0:
            rospy.logerr("Mongo process error! Exit code="+str(self._mongo_process.returncode))

        self._gone_down = True
        self._ready=False

    def _on_node_shutdown(self):
        rospy.loginfo("Shutting down datacentre")
        if self._gone_down:
            rospy.logwarn("It looks like Mongo already died. Watch out as the DB might need recovery time at next run.")
            return
        try:
            c = MongoClient(self._mongo_host,self._mongo_port)
        except pymongo.errors.ConnectionFailure, c:
            pass
        try:
            c.admin.command("shutdown")
        except pymongo.errors.AutoReconnect, a:
            pass
        
        if self.test_mode:  # remove auto-created DB in the /tmp folder
            try:
                shutil.rmtree(self.default_path)
            except Exception,e:
                rospy.logerr(e)

    def _shutdown_srv_cb(self,req):
        rospy.signal_shutdown("Shutdown request..")
        return EmptyResponse()

    def _wait_ready_srv_cb(self,req):
        while not self._ready:
            rospy.sleep(0.1)
        return EmptyResponse()

    def initialize_repl_set(self):
        c = pymongo.Connection("%s:%d" % (self._mongo_host,self._mongo_port), slave_okay=True)
        c.admin.command("replSetInitiate")
        c.close()

if __name__ == '__main__':
    server = MongoServer()

