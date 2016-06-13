#!/usr/bin/env python
# coding: utf-8
#
# Author:   Kazuto Nakashima
# URL:      https://github.com/kazuto1011
# Created:  2016-06-07

from easydict import EasyDict as edict

config = edict()
config.client = edict()
config.server = edict()

# config.client.IP = 'localhost'
config.server.IP = '192.168.4.96'

config.PORT = 49552

config.xml_path = 'C:/Users/nemuriscan/Desktop/NemuriScanLog/NemuriScanStateInfo.xml'
