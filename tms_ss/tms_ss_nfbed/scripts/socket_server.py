#!/usr/bin/env python
# coding: utf-8
#
# Author:   Kazuto Nakashima
# URL:      https://github.com/kazuto1011
# Created:  2016-06-07

import socket
import rospy as rp
import datetime
import json

from tms_msg_rs.srv import *
from tms_msg_db.msg import TmsdbStamped
from tms_msg_db.msg import Tmsdb

from config import config

#-------------------------------------------------------------------------------
# A class to monitor user's sleep state continuously
class NemuriScannerBridge:
    def __init__(self, config):
        rp.init_node('tms_ss_nfbed')

        self.address = (config.server.IP, config.PORT)
        self.server  = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(self.address)
        self.server.listen(10)
        self.db_pub = rp.Publisher('tms_db_data', TmsdbStamped, queue_size=10)
        # self.r = rp.Rate(1)

        print 'NemuriScannerBridge launched!'

    def run(self):
        print 'Waiting for connections...'
        self.client,_ = self.server.accept()

        while not rp.is_shutdown():
            # self.r.sleep()
            msg = self.client.recv(1024)
            self.call_dbwriter(msg)
            self.client.sendall('hello')

        self.client.close()

    def call_dbwriter(self, msg):
        print 'SleepState -> %s' % (msg)

        note_d = {"sleepstate": str(msg)}
        note_j = json.dumps(note_d)

        msg = TmsdbStamped()
        db = Tmsdb()
        db.time = datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")
        db.name = "person_2_moverio"
        db.id = 1002
        db.sensor = 3001
        db.place = 5001
        db.note = note_j
        msg.tmsdb.append(db)
        msg.header.stamp = rp.get_rostime() + rp.Duration(9 * 60 * 60)

        self.db_pub.publish(msg)

#-------------------------------------------------------------------------------
node = NemuriScannerBridge(config)
node.run()
