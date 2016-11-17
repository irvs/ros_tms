#!/usr/bin/env python
# coding: utf-8
import rospy as rp
import socket
import json

from tms_msg_ss.msg import Beacon

class ReceiveBeacon:
	def __init__(self):
		rp.init_node('tms_ss_beacon')
		self.server = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		self.server.bind(('',9090))
		self.pub = rp.Publisher('tms_ss_beacon',Beacon,queue_size=10)
		print 'ready'

	def run(self):
		while not rp.is_shutdown():
			rcv,addr = self.server.recvfrom(1024)
			data = json.loads(rcv)
			distance = pow(10,float(data["measuredPower"]-data["rssi"])/20.0)
			msg = Beacon()
			msg.uuid = data["uuid"]
			msg.distance = distance
			msg.accuracy = data["accuracy"]
			self.pub.publish(msg)


			print "uuid=" + data["uuid"] + " ,distance=" + str(round(distance,2)) + "m +-" + str(round(data["accuracy"],2)) + "m"

node = ReceiveBeacon()
node.run()
