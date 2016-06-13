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

config.client.IP = 'localhost'
config.server.IP = 'localhost'

config.PORT = 49552

config.xml_path = './sample_data/NemuriScanStateInfo.xml'
