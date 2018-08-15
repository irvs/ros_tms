#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from tms_msg_db.msg import Tmsdb
from tms_msg_db.srv import TmsdbGetData
from flask import Flask, jsonify, abort, make_response,request
import os
host_url = os.getenv("ROS_HOSTNAME", "localhost")



api = Flask(__name__)

def db_reader(data):
    rospy.wait_for_service('tms_db_reader')
    try:
        tms_db_reader = rospy.ServiceProxy('tms_db_reader', TmsdbGetData)
        res = tms_db_reader(data)
        return res
    except rospy.ServiceException as e:
        print "Service call failed: %s" % e
        return None

def tag_reader(data):
    temp_dbdata = Tmsdb()
    temp_dbdata.tag='「'+data+'」'
    target = db_reader(temp_dbdata)
    return target


@api.route('/get', methods=['GET'])
def get():
    response = make_response(jsonify({
        'status':'OK'
    }))
    response.statusCode = 204
    
    return response

@api.route('/post', methods=['POST'])
def post():
    if(request.headers['Content-Type']!= 'application/json'):
        print request.headers['Content-Type']
        return jsonify({
            "message":"request format is not appropriate"
        })

    print request.json
    req_word=request.json["words"]

    print(req_word)
    response = search_db(req_word)
    if not response:
        return make_response(jsonify({
            'message':'Could not find them in tms_db',
        }))
        #else:
        #    status = 503

    return make_response(jsonify({
            'message':'OK',
            'service_id':{
                "task_id":response[0],
                "robot_id":response[1],
                "user_id":response[2],
                "object_id":response[3],
                "place_id":response[4]
            }
        }))

def search_db(req_words):
    task_dic ={}
    user_dic = {}
    robot_dic = {}
    object_dic = {}
    place_dic = {}
    task_id = 0
    robot_id = 0
    object_id = 0
    user_id = 0
    place_id = 0


    for word in req_words:
        res = tag_reader(word)
        if res is None:
            return False
        for target in res.tmsdb:
            if target.type == 'task':
                task_dic[target.id] = target.announce
            elif target.type == 'robot':
                robot_dic[target.id] = target.announce
            elif target.type == 'object':
                object_dic[target.id] = target.announce
            elif target.type == 'person':
                user_dic[target.id] = target.announce
            elif target.type == 'furniture':
                place_dic[target.id] = target.announce

    if len(task_dic) == 1:
        task_id = task_dic.keys()[0]
        announce = task_dic[task_id]
    elif len(task_dic) > 1:
        print "len(task_dic) > 1"
        #未実装

    anc_list = announce.split("$")
    for anc in anc_list:
        if anc == "robot":
            if len(robot_dic) == 1:
                robot_id = robot_dic.keys()[0]
            elif len(robot_dic) > 1:
                print "len(robot_dic) > 1"
                                #未実装
            if robot_id==0:
                return False
        elif anc == "object":
            if len(object_dic) == 1:
                object_id = object_dic.keys()[0]
            elif len(object_dic) > 1:
                print "len(object_dic) > 1"
                #未実装
            if object_id==0:
                return False
        elif anc == "user":
            if len(user_dic) == 1:
                user_id = user_dic.keys()[0]
            elif len(user_dic) > 1:
                print "len(user_dic) > 1"
                #未実装
            if user_id==0:
                return False
        elif anc == "place":
            if len(place_dic) == 1:
                place_id = place_dic.keys()[0]
            elif len(place_dic) > 1:
                print "len(place_dic) > 1"
                #未実装
            if place_id==0:
                return False
    response = [task_id, robot_id, user_id, object_id, place_id]

    return response
if __name__ == "__main__":
    #api.run(host='localhost', port = 3000)
    api.run(host=host_url, port = 3000)