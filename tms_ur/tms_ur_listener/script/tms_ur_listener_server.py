#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from tms_ur_listener.msg import julius_msg
from tms_ur_speaker.srv import *
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import String
from janome.tokenizer import Tokenizer
from tms_msg_db.msg import Tmsdb
from tms_msg_db.srv import TmsdbGetData
from tms_msg_ts.srv import ts_req
import requests
import time
import subprocess
import shlex
import json
import datetime
import threading
import urllib

trigger = ['ROS-TMS']
error_msg0 = "すみません。聞き取れませんでした。"
error_msg1 = "すみません。よくわかりませんでした。"
error_msg2 = "エラーが発生したため、処理を中断します"
sid = 100000

class TmsUrListener():
    def __init__(self):
        rospy.init_node("tms_ur_listener")
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber("/pi0/julius_msg",julius_msg,self.callback, callback_args=0)
        rospy.Subscriber("/pi1/julius_msg",julius_msg,self.callback, callback_args=1)
        rospy.Subscriber("/pi2/julius_msg",julius_msg,self.callback, callback_args=2)
        rospy.Subscriber("/pi3/julius_msg",julius_msg,self.callback, callback_args=3)
        rospy.Subscriber("/pi4/julius_msg",julius_msg,self.callback, callback_args=4)
        rospy.Subscriber("/pi5/julius_msg",julius_msg,self.callback, callback_args=5)
        self.gSpeech_pi0 = rospy.Publisher("/pi0/gSpeech",Empty,queue_size=1)
        self.gSpeech_pi1 = rospy.Publisher("/pi1/gSpeech",Empty,queue_size=1)
        self.gSpeech_pi2 = rospy.Publisher("/pi2/gSpeech",Empty,queue_size=1)
        self.gSpeech_pi3 = rospy.Publisher("/pi3/gSpeech",Empty,queue_size=1)
        self.gSpeech_pi4 = rospy.Publisher("/pi4/gSpeech",Empty,queue_size=1)
        self.gSpeech_pi5 = rospy.Publisher("/pi5/gSpeech",Empty,queue_size=1)
        self.gSpeech_launched = False
        self.timer = threading.Timer(1,self.alarm)

        self.power_pub = rospy.Publisher("julius_power",Bool,queue_size=10)
        self.speaker_pub = rospy.Publisher("speaker",String,queue_size=10)
        self.tok = Tokenizer()

        f = open('/home/rts/apikey','r')
        for line in f:
            self.apikey = line.replace('\n','')

        f.close()
        print 'tms_ur_listener_server ready...'

    def alarm(self):
        while True:
            print "alarm"
            self.speaker('\sound4')
            time.sleep(1.5)
            temp_dbdata=Tmsdb()
            temp_dbdata.id = 1100
            temp_dbdata.state = 1
            target = self.db_reader(temp_dbdata)
            if target is None:
                self.announce(error_msg2)
                return
            print 'rp:'+str(target.rp)
            if target.rp>-0.2:
                break

    def julius_power(self,data,t=0):
        msg = Bool()
        msg.data = data
        time.sleep(float(t))
        self.power_pub.publish(msg)
        if data == True:
            time.sleep(1.5)
            self.speaker('\sound3')

    def launch_gSpeech(self,id):
        msg = Empty()
        if id == 0:
            self.gSpeech_pi0.publish(msg)
        elif id == 1:
            self.gSpeech_pi1.publish(msg)
        elif id == 2:
            self.gSpeech_pi2.publish(msg)
        elif id == 3:
            self.gSpeech_pi3.publish(msg)
        elif id == 4:
            self.gSpeech_pi4.publish(msg)
        else:
            self.gSpeech_pi5.publish(msg)

    def speaker(self,data):
        speak = String()
        speak.data = data
        self.speaker_pub.publish(speak)

    def announce(self,data):
        print data
        rospy.wait_for_service('speaker_srv', timeout=1.0)
        time = 0
        try:
            speak = rospy.ServiceProxy('speaker_srv',speaker_srv)
            time = speak(data)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        self.julius_power(True,time.sec)

    def db_reader(self,data):
        target=Tmsdb()
        rospy.wait_for_service('tms_db_reader')
        try:
            tms_db_reader = rospy.ServiceProxy('tms_db_reader', TmsdbGetData)
            res = tms_db_reader(data)
            target = res.tmsdb[0]
            return target
        except rospy.ServiceException as e:
            print "Service call failed: %s" % e
            return None

    def tag_reader(self,data):
        temp_dbdata = Tmsdb()
        temp_dbdata.tag='「'+data+'」'
        target = self.db_reader(temp_dbdata)
        return target

    def callback(self, data, id):
        rospy.loginfo(str(id))
        rospy.loginfo(data)
        if data.value >= 10.0: #google speech api
            self.gSpeech_launched = False
            if data.data == "":
                self.announce(error_msg0)
                return
            rospy.loginfo("get command!")
            tokens = self.tok.tokenize(data.data.decode('utf-8'))
            words = []
            verb = ''
            for token in tokens:
                print token
                if token.part_of_speech.split(',')[0] == u'動詞':
                    verb += token.base_form.encode('utf-8')
                elif token.part_of_speech.split(',')[0] == u'名詞':
                    if token.base_form.encode('utf-8') != "*":
                        words.append(token.base_form.encode('utf-8'))
                    else:
                        words.append(token.surface.encode('utf-8'))
            if verb != '':
                words.append(verb)
            if "言う" in words: #「〇〇に行って」が「〇〇に言って」と認識される
                words.append("行く")
            print str(words).decode('string-escape')
            task_id = 0
            robot_id = 0
            object_id = 0
            user_id = 1100
            place_id = 0
            announce = ""
            robot_name = ""
            object_name = ""
            user_name = "太郎さん"
            place_name = ""
            other_words = []

            for word in words:
                target = self.tag_reader(word)
                if target is None:
                    self.announce(error_msg2)
                    return
                if target.type == 'task':
                    task_id = target.id
                    announce = target.announce
                elif target.type == 'robot':
                    robot_id = target.id
                    robot_name = target.announce
                elif target.type == 'object':
                    object_id = target.id
                    object_name = target.announce
                elif target.type == 'person':
                    user_id = target.id
                    user_name = target.announce
                elif target.type == 'furniture':
                    place_id = target.id
                    place_name = target.announce
                else:
                    other_words.append(word)

            print "task_id:" + str(task_id)
            print "robot_id:" + str(robot_id)
            print "object_id:" + str(robot_id)
            print "user_id:" + str(user_id)
            print "place_id:" + str(place_id)

            if task_id == 0:
                print 'ask docomo Q&A api'
                print data.data
                urlenc = urllib.quote(data.data)
                args = "curl -s 'https://api.apigw.smt.docomo.ne.jp/knowledgeQA/v1/ask?APIKEY=" + self.apikey + "&q=" + urlenc + "'"
                ret = subprocess.check_output(shlex.split(args))
                json_dict = json.loads(ret,"utf-8")
                announce = "すみません、わかりませんでした。"
                if "message" in json_dict:
                    print json_dict["message"]["textForDisplay"]
                    announce = json_dict["message"]["textForSpeech"]
                self.announce(announce)
                return
            elif task_id == 8100: #search_object
                place_id = 0
                place_name = ""
                temp_dbdata = Tmsdb()
                temp_dbdata.id = object_id
                temp_dbdata.state = 1
                target = self.db_reader(temp_dbdata)
                if target is None:
                    self.announce(error_msg2)
                    return
                place_id = target.place

                temp_dbdata = Tmsdb()
                temp_dbdata.id = place_id + sid
                target = self.db_reader(temp_dbdata)
                if target is None:
                    self.announce(error_msg2)
                    return
                place_name = target.announce

                if object_name == "" or place_name == "":
                    self.announce(error_msg1)
                    return

                anc_list = announce.split("$")
                announce = ""
                for anc in anc_list:
                    if anc == "object":
                        announce += object_name
                    elif anc == "place":
                        announce += place_name
                    else:
                        announce += anc
                self.announce(announce)
            elif task_id == 8101: #weather_forecast
                place = "福岡市"
                date = ""
                weather = ""
                for word in other_words:
                    if word in ['今日','明日','明後日','あさって']:
                        date = word
                if date == "":
                    self.announce(error_msg1)
                    return

                args = "curl -s http://weather.livedoor.com/forecast/webservice/json/v1\?city\=400010"
                ret = subprocess.check_output(shlex.split(args))
                json_dict = json.loads(ret,"utf-8")
                if "forecasts" in json_dict:
                    if date == '今日':
                        weather = json_dict["forecasts"][0]["telop"].encode('utf-8')
                    elif date == '明日':
                        weather = json_dict["forecasts"][1]["telop"].encode('utf-8')
                    elif date == '明後日' or date == 'あさって':
                        weather = json_dict["forecasts"][2]["telop"].encode('utf-8')
                if weather == "":
                    self.announce(error_msg1)
                    return

                anc_list = announce.split("$")
                announce = ""
                for anc in anc_list:
                    if anc == "place":
                        announce += place
                    elif anc == "date":
                        announce += date
                    elif anc == "weather":
                        announce += weather
                    else:
                        announce += anc
                self.announce(announce)

            elif task_id == 8102: #set_alarm
                today = datetime.datetime.today()
                print 'now:' + today.strftime("%Y/%m/%d %H:%M:%S")
                if today.hour < 6:
                    date = 0
                else:
                    date = 1
                hour = -1
                minute = 0
                for i,word in enumerate(other_words):
                    if word == "今日":
                        date = 0
                    elif word == "明日" and today.hour > 6:
                        date = 1
                    elif word in ["時","時半"] and i>0:
                        if words[i-1].isdigit():
                            hour = int(words[i-1])
                            if word == "時半":
                                minute = 30
                            if i>1 and words[i-2] == "午後" and hour <=12:
                                hour += 12
                            elif i>1 and words[i-2] == "夜" and hour <=12 and hour>=6:
                                hour += 12
                    elif word == "分":
                        if words[i-1].isdigit():
                            minute = int(words[i-1])
                print "d:"+str(date)+" h:"+str(hour)+" m:"+str(minute)
                if hour == -1:
                    self.announce(error_msg1)
                    return

                tgt_time = datetime.datetime(today.year,today.month,today.day,hour,minute,0,0)
                tgt_time += datetime.timedelta(1)
                print 'tgt_time:' + tgt_time.strftime("%Y/%m/%d %H:%M:%S")
                offset = tgt_time - today
                print 'offset_sec:' + str(offset.seconds)

                if offset.seconds < 0:
                    self.announce(error_msg1)
                    return

                self.timer = threading.Timer(15,self.alarm)#(offset.seconds,self.alarm)
                self.timer.start()

                anc_list = announce.split("$")
                announce = ""
                for anc in anc_list:
                    if anc == "date":
                        announce += str(tgt_time.month)+"月"+str(tgt_time.day)+"日"
                    elif anc == "time":
                        announce += str(tgt_time.hour)+"時"+str(tgt_time.minute)+"分"
                    else:
                        announce += anc

                self.announce(announce)
            elif task_id == 8103:
                url = "http://192.168.100.101/codemari_kyudai/CodemariServlet?deviceID=9999&locale=ja&cmd=%251CFLP%"
                onoff = ""
                if "つける" in other_words:
                    print "light on"
                    onoff = "付け"
                    url += "2003"
                elif "消す" in other_words:
                    print "light off"
                    onoff = "消し"
                    url += "2005"
                else:
                    self.announce(error_msg1)
                    return

                anc_list = announce.split("$")
                announce = ""
                for anc in anc_list:
                    if anc == "onoff":
                        announce += onoff
                    else:
                        announce += anc
                self.announce(announce)

                res = requests.get(url)
                print res.text

            else: #robot_task
                anc_list = announce.split("$")
                announce = ""
                for anc in anc_list:
                    if anc == "robot":
                        if robot_id==0:
                            self.announce(error_msg1)
                            return
                        announce += robot_name
                    elif anc == "object":
                        if object_id==0:
                            self.announce(error_msg1)
                            return
                        announce += object_name
                    elif anc == "user":
                        if user_id==0:
                            self.announce(error_msg1)
                            return
                        announce += user_name
                    elif anc == "place":
                        if place_id==0:
                            self.announce(error_msg1)
                            return
                        announce += place_name
                    else:
                        announce += anc

                print 'send command'
                try:
                    rospy.wait_for_service('tms_ts_master', timeout=1.0)
                except rospy.ROSException:
                    print "tms_ts_master is not work"

                try:
                    tms_ts_master = rospy.ServiceProxy('tms_ts_master',ts_req)
                    res = tms_ts_master(0,task_id,robot_id,object_id,user_id,place_id,0)
                    print res
                except rospy.ServiceException as e:
                    print "Service call failed: %s" % e

                self.announce(announce)

        elif data.value > 0.8: #Julius
            if data.data in trigger:
                if self.gSpeech_launched == False:
                    self.gSpeech_launched = True
                    rospy.loginfo("call trigger on raspi:%d",id)
                    rospy.loginfo("kill julius!!")
                    self.julius_power(False)
                    self.speaker("\sound1")
                    time.sleep(0.5)
                    self.launch_gSpeech(id)

    def shutdown(self):
        rospy.loginfo("Stopping the node")

if __name__ == '__main__':
    try:
        TmsUrListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("tms_ur_listener node terminated.")
