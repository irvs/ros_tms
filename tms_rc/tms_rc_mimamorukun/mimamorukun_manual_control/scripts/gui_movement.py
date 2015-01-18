!/usr/bin/env python
# -*- coding:utf-8 -*-

import os,sys
from PyQt4 import QtGui, QtCore
import rospy
#import roslib
from tms_msg_rs.srv import *
from tms_msg_db.msg import TmsdbStamped
from tms_msg_db.msg import Tmsdb
import tms_msg_db.srv
import tms_msg_rp.srv

SCRIPT_PATH = os.path.abspath(os.path.dirname(__file__))
MAP_PATH = '/images/map.png'
TGT_PATH = '/images/tgt_pos.png'
WC_PATH = '/images/wc.png'

MAP_ORIGN = QtCore.QPoint(10.0, 450.0)
MAP_SIZE = QtCore.QPoint(780.0, 440.0)
ROOM_SIZE = QtCore.QPoint(8000.0, 4500.0)


class MainWidget(QtGui.QWidget):

    def __init__(self):
        super(MainWidget, self).__init__()
        self.initUI()

    def initUI(self):
        self.tgt_pos = QtCore.QPoint(0, 0)
        self.wc_mark_pos = QtCore.QPoint(0, 0)
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

        self.tgt_lbl = QtGui.QLabel(self)
        self.tgt_lbl.resize(50, 50)
        tgt_pmp = QtGui.QPixmap(50, 50)
        tgt_pmp = QtGui.QPixmap(SCRIPT_PATH+TGT_PATH).scaled(
            self.tgt_lbl.size(),
            aspectRatioMode=QtCore.Qt.KeepAspectRatio,
            transformMode=QtCore.Qt.SmoothTransformation)
        self.tgt_lbl.setPixmap(tgt_pmp)

        self.wc_lbl = QtGui.QLabel(self)
        self.wc_lbl.resize(30, 30)
        wc_pmp = QtGui.QPixmap(30, 30)
        wc_pmp = QtGui.QPixmap(SCRIPT_PATH+WC_PATH).scaled(
            self.wc_lbl.size(),
            aspectRatioMode=QtCore.Qt.KeepAspectRatio,
            transformMode=QtCore.Qt.SmoothTransformation)
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
        wc_pos = QtCore.QPoint(0, 0)
        try:
            srv_client = rospy.ServiceProxy("/tms_db_reader/dbreader",
                                            tms_msg_db.srv.TmsdbGetData)
            req = tms_msg_db.srv.TmsdbGetDataRequest()
            req.tmsdb.id = 2007
            req.tmsdb.sensor = 3001
            res = srv_client(req)
            if 0 < len(res.tmsdb):
                wc_pos = QtCore.QPoint(res.tmsdb[0].x, res.tmsdb[0].y)
                print (wc_pos.x(), wc_pos.y())
                wc_mark_pos = QtCore.QPoint(
                    wc_pos.x()/ROOM_SIZE.x()*MAP_SIZE.x()+MAP_ORIGN.x(),
                    -wc_pos.y()/ROOM_SIZE.y()*MAP_SIZE.y()+MAP_ORIGN.y())
                self.wc_lbl.move(wc_mark_pos)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e)

    def startMoving(self):
        try:
            srv_client = rospy.ServiceProxy("/rp_cmd",
                                            tms_msg_rp.srv.rp_cmd)
            req = tms_msg_rp.srv.rp_cmdRequest()
            req.command = 9001   # move
            req.type = True      # not sim, in Real World.But this param isn't used yet
            req.robot_id = 2007  # ID of mimamorukun
            req.arg = [-1, self.tgt_pos.x(), self.tgt_pos.y(), 0]
            res = srv_client(req)
            print ("cmd result:", res.result)
        except rospy.ServiceException, e:
            print ("Service call failed: %s" % e)

    def draw(self):
        pass

    def updateTgtPos(self, event):
        self.tgt_pos = QtCore.QPoint(
            (event.pos().x()*1.0-MAP_ORIGN.x())/MAP_SIZE.x() * ROOM_SIZE.x(),
            (-event.pos().y()*1.0+MAP_ORIGN.y())/MAP_SIZE.y() * ROOM_SIZE.y())
        tgt_mark_pos = QtCore.QPoint(event.pos().x(), event.pos().y())
        # print self.tgt_pos,
        self.tgt_lbl.move(tgt_mark_pos - self.tgt_lbl.rect().center())


def main():
    print ("\x1b[32mHello World\x1b[39m")
    app = QtGui.QApplication(sys.argv)
    mnw = MainWidget()
    mnw.bt_quit.clicked.connect(app.quit)

    sys.exit(app.exec_())

    ###init ROS
    rospy.init_node('tms_ss_vs_viewer')
    print ("finding DB service server")
    rospy.wait_for_service('/tms_db_reader/dbreader')
    print ("found DB service server")

if __name__ == '__main__':
    main()
