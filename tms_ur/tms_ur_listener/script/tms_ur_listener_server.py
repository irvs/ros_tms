#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from tms_ur_listener.msg import julius_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from janome.tokenizer import Tokenizer
from tms_msg_db.msg import Tmsdb
from tms_msg_db.srv import TmsdbGetData
from tms_msg_ts.srv import ts_req

class TmsUrListener():
    def __init__(self):
        rospy.init_node("tms_ur_listener")
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber("julius_msg",julius_msg,self.callback)
        self.power_pub = rospy.Publisher("julius_power",Bool,queue_size=10)
        self.speaker_pub = rospy.Publisher("speaker",String,queue_size=10)
        self.t = Tokenizer()
        self.robot_name = ''

    def callback(self, data):
        rospy.loginfo(data)
        if data.value >= 10.0:
            rospy.loginfo("get command!")
            tokens = self.t.tokenize(data.data.decode('utf-8'))
            nouns = []
            verb = ''
            for token in tokens:
                print token
                if token.part_of_speech.split(',')[0] == u'名詞':
                    nouns.append(token.base_form.encode('utf-8'))
                if token.part_of_speech.split(',')[0] == u'動詞':
                    verb += token.base_form.encode('utf-8')
            print str(nouns).decode('string-escape')
            print verb
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
            if verb != '':
                temp_dbdata = Tmsdb()
                target = Tmsdb()
                temp_dbdata.tag=verb
                rospy.wait_for_service('tms_db_reader')
                try:
                    tms_db_reader = rospy.ServiceProxy('tms_db_reader', TmsdbGetData)
                    res = tms_db_reader(temp_dbdata)
                    target = res.tmsdb[0]
                    task_id = target.id
                    announce = target.announce
                except rospy.ServiceException as e:
                    print "Service call failed: %s" % e

            if self.robot_name != '':
                temp_dbdata = Tmsdb()
                target = Tmsdb()
                temp_dbdata.tag=self.robot_name
                rospy.wait_for_service('tms_db_reader')
                try:
                    tms_db_reader = rospy.ServiceProxy('tms_db_reader', TmsdbGetData)
                    res = tms_db_reader(temp_dbdata)
                    target = res.tmsdb[0]
                    robot_id = target.id
                    robot_name = target.announce
                except rospy.ServiceException as e:
                    print "Service call failed: %s" % e

            for noun in nouns:
                temp_dbdata = Tmsdb()
                target = Tmsdb()
                temp_dbdata.tag=noun
                rospy.wait_for_service('tms_db_reader')
                try:
                    tms_db_reader = rospy.ServiceProxy('tms_db_reader', TmsdbGetData)
                    res = tms_db_reader(temp_dbdata)
                    target = res.tmsdb[0]
                    if target.type == 'object':
                        object_id = target.id
                        object_name = target.announce
                    elif target.type == 'person':
                        user_id = target.id
                        user_name = target.announce
                    elif target.type == 'furniture':
                        place_id = target.id
                        place_name = target.announce
                except rospy.ServiceException as e:
                    print "Service call failed: %s" % e

            anc_list = announce.split("$")
            announce = ""
            for anc in anc_list:
                if anc == "robot":
                    announce += robot_name
                elif anc == "object":
                    announce += object_name
                elif anc == "user":
                    announce += user_name
                elif anc == "place":
                    announce += place_name
                else:
                    announce += anc

            print announce
            speak = String()
            speak.data = announce
            self.speaker_pub.publish(speak)

            print 'send command'
            try:
                rospy.wait_for_service('tms_ts_master', timeout=2.0)
            except rospy.ROSException:
                print "tms_ts_master timeout"
            try:
                tms_ts_master = rospy.ServiceProxy('tms_ts_master',ts_req)
                res = tms_ts_master(0,task_id,robot_id,object_id,user_id,place_id,0)
                print res
            except rospy.ServiceException as e:
                print "Service call failed: %s" % e

        elif data.value > 0.8:
            if data.data in ['スマートパル','見守る君','TMS']:
                rospy.loginfo("call robot name")
                rospy.loginfo("kill julius!!")
                self.robot_name = data.data
                msg = Bool()
                msg.data = False
                self.power_pub.publish(msg)
                speak = String()
                speak.data = "\sound1"
                self.speaker_pub.publish(speak)

    def shutdown(self):
        rospy.loginfo("Stopping the node")

if __name__ == '__main__':
    try:
        TmsUrListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("tms_ur_listener node terminated.")
