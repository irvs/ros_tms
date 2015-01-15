#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os,sys
# import platform
# import PyQt4.QtCore as QtGui,QtCore
from PyQt4 import QtGui, QtCore
import json
import datetime
import time
import subprocess
import rospy
#import roslib
from tms_msg_rs.srv import *
from tms_msg_db.msg import TmsdbStamped
from tms_msg_db.msg import Tmsdb
# from tms_msg_db.srv import TmsdbGetData
import tms_msg_db.srv
#include <tms_msg_db/TmsdbGetData.h>

SCRIPT_PATH = os.path.abspath(os.path.dirname(__file__))
MAP_PATH = '/images/map.png'
# WC_PATH = '/images/wc.png'

MAP_ORIGN = QtCore.QPoint(10, 450)
MAP_SIZE = QtCore.QPoint(780, 440)
ROOM_SIZE = QtCore.QPoint(8000, 4500)


class MainWidget(QtGui.QWidget):

    def __init__(self):
        super(MainWidget, self).__init__()
        self.initUI()

    def initUI(self):
        self.cur_pos_x = 0
        self.cur_pos_y = 0
        self.tgt_pos_x = 0
        self.tgt_pos_y = 0
        self.updateCurPos()

        map_lbl = QtGui.QLabel(self)
        map_lbl.setGeometry(0, 0, 800, 480)
        map_lbl.setAlignment(QtCore.Qt.AlignTop)
        map_pmp = QtGui.QPixmap(SCRIPT_PATH+MAP_PATH).scaled(
            map_lbl.size(),
            aspectRatioMode=QtCore.Qt.KeepAspectRatio,
            transformMode=QtCore.Qt.SmoothTransformation)
        map_lbl.setPixmap(map_pmp)
        map_lbl.mousePressEvent = self.updateTgtPos

        self.wc_lbl = QtGui.QLabel(self)
        self.wc_lbl.resize(40, 40)
        wc_pmp = QtGui.QPixmap(50, 50)
        wc_pantr = QtGui.QPainter(wc_pmp)
        wc_pantr.setOpacity(1.0)
        # wc_pantr.setBackgroundMode(QtCore.Qt.OpaqueMode)
        # wc_pantr.setBrush(QtGui.QColor(123, 123, 123, 0))
        # wc_pantr.drawRect(0, 0, 50, 50)
        wc_pantr.setBrush(QtGui.QBrush(QtCore.Qt.black))
        wc_pantr.drawEllipse(0, 0, 50, 50)
        self.wc_lbl.setPixmap(wc_pmp)

        bt_move = QtGui.QPushButton("MOVE", parent=self)
        bt_move.move(130, 500)
        bt_move.clicked.connect(self.startMoving)

        self.bt_quit = QtGui.QPushButton("QUIT", parent=self)
        self.bt_quit.move(530, 500)

        self.timer = QtCore.QTimer(parent=self)
        self.timer.setInterval(100)  # 100ms
        self.timer.timeout.connect(self.timerLoop)
        self.timer.start()

        self.move(10, 10)
        self.resize(800, 550)
        self.setWindowTitle("testTittle")
        self.show()

    def timerLoop(self):
        self.updateCurPos()
        self.draw()

    def updateCurPos(self):
        try:
            db_client = rospy.ServiceProxy("/tms_db_reader/dbreader",
                                           tms_msg_db.srv.TmsdbGetData)
            req = tms_msg_db.srv.TmsdbGetDataRequest()
            req.tmsdb.id = 2007
            req.tmsdb.sensor = 3001
            res = db_client(req)
            if 0 < len(res.tmsdb):
                self.cur_pos_x, self.cur_pos_y = res.tmsdb[0].x, res.tmsdb[0].y
                print res.tmsdb[0].x, res.tmsdb[0].y
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def startMoving(self):
        try:
            db_client = rospy.ServiceProxy("/rp_cmd",
                                           tms_msg_db.srv.TmsdbGetData)
            req = tms_msg_db.srv.TmsdbGetDataRequest()
            req.tmsdb.id = 2007
            req.tmsdb.sensor = 3001
            res = db_client(req)
            if 0 < len(res.tmsdb):
                self.cur_pos_x, self.cur_pos_y = res.tmsdb[0].x, res.tmsdb[0].y
                print res.tmsdb[0].x, res.tmsdb[0].y
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def draw(self):
        pass

    def updateTgtPos(self, event):
        self.tgt_pos_x, self.tgt_pos_y = (event.pos().x(), event.pos().y())
        print self.tgt_pos_x, self.tgt_pos_y,
        self.wc_lbl.move(QtCore.QPoint(self.tgt_pos_x, self.tgt_pos_y) - self.wc_lbl.rect().center())


def main():
    print "\x1b[32mHello World\x1b[39m"
    app = QtGui.QApplication(sys.argv)
    mnw = MainWidget()
    mnw.bt_quit.clicked.connect(app.quit)

    sys.exit(app.exec_())

    ###init ROS
    rospy.init_node('tms_ss_vs_viewer')
    print ("finding DB service server")
    rospy.wait_for_service('/tms_db_reader/dbreader')
    print ("found DB service server")

    # try:
    #     db_client = rospy.ServiceProxy("/tms_db_reader/dbreader", tms_msg_db.srv.TmsdbGetData)
    #     req = tms_msg_db.srv.TmsdbGetDataRequest()
    #     req.tmsdb.id = 3018
    #     req.tmsdb.sensor = 3018
    #     res = db_client(req)
    #     if 0 < len(res.tmsdb):
    #         print (res.tmsdb[0].note)
    # except rospy.ServiceException, e:
    #     print "Service call failed: %s" % e
    # while True:
    #     pass


if __name__ == '__main__':
    main()
