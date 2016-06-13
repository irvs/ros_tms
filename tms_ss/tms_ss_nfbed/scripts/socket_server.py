#!/usr/bin/env python
# coding: utf-8
#
# Author:   Kazuto Nakashima
# URL:      https://github.com/kazuto1011
# Created:  2016-06-07

import socket
from config import config

#-------------------------------------------------------------------------------
# A class to monitor user's sleep state continuously
class NemuriScannerBridge:
    def __init__(self, config):
        self.address = (config.server.IP, config.PORT)
        self.server  = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(self.address)
        self.server.listen(10)
        print 'NemuriScannerBridge launched!'

    def run(self):
        print 'Waiting for connections...'
        self.client,_ = self.server.accept()
        while True:
            msg = self.client.recv(1024)
            self.call_dbwriter(msg)
            self.client.sendall('hello')
        self.client.close()

    def call_dbwriter(self, msg):
        print 'SleepState -> %s' % (msg)

#-------------------------------------------------------------------------------
node = NemuriScannerBridge(config)
node.run()
