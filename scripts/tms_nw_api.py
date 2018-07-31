#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from flask import Flask, jsonify, abort, make_response

api = Flask(__name__)

@api.route('/get', methods=['GET'])
def get():
    response = make_response(jsonify({
        'status':'OK'
    }))
    response.statusCode = 204
    
    return response

@api.route('/post', methods=['POST'])
def post():
    if(request.form != null):
        robot_name = request.form['robot']
        task_name = request.form['task']
        user_name = request.form['user']
        object_name = request.form['object']
        place_name = request.form['name']

    print robot_name
    print task_name
    print user_name
    print object_name
    print place_name

    if hoge():
        status = 200
    else:
        if hogehoge():
            status = 500
        else:
            status = 503
    return make_response(jsonify({
            'status':'OK',
            'statusCode':status
        }))

def hoge():
    return true

if __name__ == "__main__":
    api.run(host='localhost', port = 3000)