#!/usr/bin/env python
# coding: utf-8
#
# Author:   Kazuto Nakashima
# URL:      https://github.com/kazuto1011
# Created:  2016-06-07

import socket, time
from config import config
import xml.etree.ElementTree as ET

#-------------------------------------------------------------------------------
# A class to monitor user's sleep state continuously
class NemuriScanner():
    def __init__(self, config):
        self.xml     = config.xml_path
        self.address = (config.server.IP, config.PORT)
        self.client  = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect(self.address)
        print 'NemuriScanner launched!'

    def run(self):
        while True:
            self.sleep_state = self.parse()
            self.result = self.call(self.sleep_state)
            print self.result
            time.sleep(1)

    def parse(self):
        root = ET.parse(self.xml).getroot()
        return root.find('.//State').text

    def call(self, msg):
        self.client.send(msg)
        return self.client.recv(4096)

#-------------------------------------------------------------------------------
node = NemuriScanner(config)
node.run()
