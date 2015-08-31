#!/usr/bin/env python

import roslib; roslib.load_manifest('mongodb_store')
import rospy
import sys
import os
import collections
import json
import xmlrpclib
from bson.binary import Binary

import mongodb_store.util

from mongodb_store.srv import *
from std_srvs.srv import *
import rosparam

if not mongodb_store.util.check_for_pymongo():
    sys.exit(1)

import pymongo
import pymongo.son_manipulator
MongoClient = mongodb_store.util.import_MongoClient()

class MongoTransformer(pymongo.son_manipulator.SONManipulator):
    def __init__(self):
        pass
    
    def transform_incoming(self, son, collection):
        if isinstance(son, list):
            return self.transform_incoming_list(son, collection)
        elif isinstance(son, dict):
            for (key, value) in son.items():
                son[key] = self.transform_incoming(value, collection)
        elif isinstance(son, xmlrpclib.Binary):
            return {'__xmlrpclib_object':'xmlrpclib.Binary',
                   'data': Binary(son.data)}
        return son

    def transform_incoming_list(self, lst, collection):
        new_lst = map(lambda x: self.transform_incoming(x, collection),
                  lst)
        return new_lst
    
    def transform_outgoing(self, son, collection):
        if isinstance(son, list):
            return self.transform_outgoing_list(son, collection)
        elif isinstance(son, dict):
            for (key, value) in son.items():
                son[key] = self.transform_outgoing(value, collection)
    
            if "__xmlrpclib_object" in son:
                if son["__xmlrpclib_object"] == "xmlrpclib.Binary":
                    b = xmlrpclib.Binary(son['data'])
                    return b
                else:
                    raise Exception("Unhandled xmlrpclib type.")
            else:
                return son
        return son
    
    def transform_outgoing_list(self, lst, collection):
        new_lst = map(lambda x: self.transform_outgoing(x, collection),
                  lst)
        return new_lst
    
class ConfigManager(object):
    def __init__(self):
        rospy.init_node("config_manager")
        rospy.on_shutdown(self._on_node_shutdown)

        use_daemon = rospy.get_param('mongodb_use_daemon', False)
        db_host = rospy.get_param('mongodb_host')
        db_port = rospy.get_param('mongodb_port')
        if use_daemon:
            is_daemon_alive = mongodb_store.util.check_connection_to_mongod(db_host, db_port)
            if not is_daemon_alive:
                sys.exit(1)
        else:
            if not mongodb_store.util.wait_for_mongo():
                sys.exit(1)
        
        self._mongo_client = MongoClient(db_host, db_port)

        self._database=self._mongo_client.config
        self._database.add_son_manipulator(MongoTransformer())

        # Load the default settings from the defaults/ folder
        try:
            path = rospy.get_param("~defaults_path")
            if len(path)==0:
                raise 
        except:
            rospy.loginfo("Default parameters path not supplied, assuming none.")
        else:
            if path.startswith("pkg://"):
                parts = path.split("//")
                parts=parts[1].split("/",1)
                pkg=parts[0]
                pkg_dir=parts[1]
                try:
                    path = os.path.join(roslib.packages.get_pkg_dir(pkg), pkg_dir)
                except roslib.packages.InvalidROSPkgException, e:
                    rospy.logerr("Supplied defaults path '%s' cannot be found. \n"%path +
                                 "The ROS package '%s' could not be located."%pkg)
                    sys.exit(1)
            if not os.path.isdir(path):
                rospy.logwarn("Defaults path '%s' does not exist."%path)
                sys.exit(1)
            try:
                files = os.listdir(path)
            except OSError, e:
                rospy.logerr("Can't list defaults directory %s. Check permissions."%path)
                sys.exit(1)
            defaults=[]  # a list of 3-tuples, (param, val, originating_filename)
            def flatten(d, c="", f_name="" ):
                l=[]
                for k, v in d.iteritems():
                    if isinstance(v, collections.Mapping):
                        l.extend(flatten(v,c+"/"+k, f_name))
                    else:
                        l.append((c+"/"+k, v, f_name))
                return l

            for f in files:
                if not f.endswith(".yaml"):
                    continue
                params = rosparam.load_file(os.path.join(path,f))
                rospy.loginfo("Found default parameter file %s" % f)
                for p, n in params:
                    defaults.extend(flatten(p,c="",f_name=f))

            # Copy the defaults into the DB if not there already
            defaults_collection = self._database.defaults
            for param,val,filename in defaults:
                existing = defaults_collection.find_one({"path":param}, manipulate=False)
                if existing is None:
                    rospy.loginfo("New default parameter for %s"%param)
                    defaults_collection.insert({"path":param,
                                                "value":val,
                                                "from_file":filename})
                elif existing["from_file"]!=filename:
                    rospy.logerr("Two defaults parameter files have the same key:\n%s and %s, key %s"%
                                 (existing["from_file"],filename,param))
                    # Delete the entry so that it can be fixed...
                    defaults_collection.remove(existing)
                    rospy.signal_shutdown("Default parameter set error")
                else: #if str(existing_value) != str(val):
                    existing_value = self._database._fix_outgoing(existing['value'], defaults_collection)
                    for i,j in zip(str(existing_value),str(val)):
                        if i !=j:
                            break
                    else:
                        if len(str(existing_value)) == len(str(val)):
                            continue

                    rospy.loginfo("Updating stored default for %s"%param)
                    new={}
                    new.update(existing)
                    new['value']=val
                    defaults_collection.update(existing, new, manipulate=True)
                
                
        # Load the settings onto the ros parameter server
        defaults_collection = self._database.defaults
        local_collection = self._database.local
        for param in defaults_collection.find():
            name=param["path"]
            val=param["value"]
            if local_collection.find_one({"path":name}) is None:
                rospy.set_param(name,val)
        for param in local_collection.find():
            name=param["path"]
            val=param["value"]
            rospy.set_param(name,val)

        
        # Advertise ros services for parameter setting / getting
        self._getparam_srv = rospy.Service("/config_manager/get_param",
                                           GetParam,
                                           self._getparam_srv_cb)
        self._setparam_srv = rospy.Service("/config_manager/set_param",
                                           SetParam,
                                           self._setparam_srv_cb)
        self._saveparam_srv = rospy.Service("/config_manager/save_param",
                                           SetParam,
                                           self._saveparam_srv_cb)
        
        #self._list_params()
        
        # Start the main loop
        rospy.spin()

    """
    debug function, prints out all parameters known
    """
    def _list_params(self):
        print "#"*10
        print "Defaults:"
        print
        for param in self._database.defaults.find():
            name=param["path"]
            val=param["value"]
            filename=param["from_file"]
            print name, " "*(30-len(name)),val," "*(30-len(str(val))),filename
        print
        
        
    def _on_node_shutdown(self):
        self._mongo_client.disconnect()

    # Could just use the ros parameter server to get the params
    # but one day might not back onto the parameter server...
    def _getparam_srv_cb(self,req):
        response = GetParamResponse()
        config_db = self._mongo_client.config
        value = config_db.local.find_one({"path":req.param_name})
        if value is None:
            value = config_db.defaults.find_one({"path":req.param_name})
            if value is None:
                response.success=False
                return response
        response.success=True
        response.param_value=str(value["value"])
        return response

    """
    Set the local site-specific parameter.
    """
    def _setparam_srv_cb(self,req):
        print ("parse json")
        new = json.loads(req.param)
        if not (new.has_key("path") and new.has_key("value")):
            rospy.logerr("Trying to set parameter but not giving full spec")
            return SetParamResponse(False)
        config_db_local = self._database.local
        value = config_db_local.find_one({"path":new["path"]}, manipulate=False)
        if value is None:
            # insert it
            config_db_local.insert(new)
        else:
            # update it
            new['_id']=value['_id']
            config_db_local.update(value,new, manipulate=True)
            pass
        return SetParamResponse(True)

    # This will take the current value from the rosparam server and save it into the DB
    def _saveparam_srv_cb(self,req):
        if not rospy.has_param(req.param):
            rospy.logerr("Trying to set a parameter from ros parameter server, but it is not on server.")
            return SetParamResponse(False)
        val=rospy.get_param(req.param)
        new={}
        new['path']=str(req.param)
        new['value']=val
        config_db_local = self._database.local
        value = config_db_local.find_one({"path":new["path"]}, manipulate=False)
        if value is None:
            # insert it
            config_db_local.insert(new)
        else:
            # update it
            new['_id']=value['_id']
            config_db_local.update(value, new, manipulate=True)

        return SetParamResponse(True)
   

if __name__ == '__main__':
    server = ConfigManager()

