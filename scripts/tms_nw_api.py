#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from tms_msg_db.msg import Tmsdb
from tms_msg_db.srv import TmsdbGetData
from flask import Flask, jsonify, abort, make_response,request

api = Flask(__name__)
task_dic = {}
robot_dic = {}
object_dic = {}
user_dic = {1100:"太郎さん"}
place_dic = {}

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
        status = 400
        return jsonify({
            'status':'error',
            'statusCode':status
        })

    print request.json
    req_word=['','','','','']
    req_word[0] = request.json['robot']
    req_word[1] = request.json['task']
    req_word[2] = request.json['user']
    req_word[3] = request.json['object']
    req_word[4] = request.json['place']

            
    if search_db(req_word):
        status = 200
    else:
        status = 500
        print status
        return make_response(jsonify({
            'status':'error',
            'statusCode':status
        }))
        #else:
        #    status = 503
    print status
    return make_response(jsonify({
            'status':'OK',
            'statusCode':status
        }))

def search_db(req_word):
    for word in req_word:
        if word != "":
            temp_dbdata = Tmsdb()
            res = tag_reader(word)
            if len(res.tmsdb)==1 and res.tmsdb[0].id == 0:
                return False
    return True
if __name__ == "__main__":
    api.run(host='localhost', port = 3000)