# #!/usr/bin/env python
# # -*- coding:utf-8 -*-
#
# import rospy
# from pypozyx import *
# from visualization_msgs.msg import Marker
#
# class tms_ss_pozyx():
#     def __init__(self):
#         rospy.init_node("tms_ss_pozyx")
#         rospy.on_shutdown(self.shutdown)
#         self.pos_pub = rospy.Publisher('pozyx',Marker,queue_size=10)
#
#         serial_port = get_first_pozyx_serial_port()
#         self.pozyx = PozyxSerial(serial_port)
#
#         self.tags = [0x6077]
#         self.anchors = [DeviceCoordinates(0x605B, 1, Coordinates(4500,2884,995)),
#                         DeviceCoordinates(0x685D, 1, Coordinates(4805,146,2192)),
#                         DeviceCoordinates(0x603B, 1, Coordinates(8033,143,452)),
#                         DeviceCoordinates(0x683B, 1, Coordinates(11179,153,1370)),
#                         DeviceCoordinates(0x6822, 1, Coordinates(9778,5824,2533)),
#                         DeviceCoordinates(0x6031, 1, Coordinates(7531,5571,85)),
#                         DeviceCoordinates(0x680A, 1, Coordinates(6316,6215,1798)),
#                         DeviceCoordinates(0x6173, 1, Coordinates(7365,2246,3115))]
#         self.algorithm = POZYX_POS_ALG_UWB_ONLY
#         self.dimension = POZYX_3D
#         self.setup()
#         self.loop()
#
#     def shutdown(self):
#         rospy.loginfo("stopping the node")
#
#     def setup(self):
#         self.setAnchorsManual()
#         self.printPublishAnchorConfiguration()
#
#     def loop(self):
#         while not rospy.is_shutdown():
#             for tag in self.tags:
#                 position = Coordinates()
#                 q = Quaternion()
#                 status = self.pozyx.doPositioning(position, self.dimension, 0, self.algorithm, remote_id=tag)
#                 self.pozyx.getQuaternion(q,remote_id=tag)
#                 if status == POZYX_SUCCESS:
#                     self.printPublishPosition(position, q, tag)
#                     msg = Marker()
#                     msg.header.frame_id = "world_link"
#                     msg.header.stamp = rospy.Time.now()
#                     msg.type = Marker.SPHERE
#                     msg.pose.position.x = position.x*0.001
#                     msg.pose.position.y = position.y*0.001
#                     msg.pose.position.z = position.z*0.001
#                     msg.pose.orientation.x = q.x
#                     msg.pose.orientation.y = q.y
#                     msg.pose.orientation.z = q.z
#                     msg.pose.orientation.w = q.w
#                     msg.scale.x = 0.5
#                     msg.scale.y = 0.5
#                     msg.scale.z = 0.5
#                     msg.color.a = 0.5
#                     msg.color.r = 1.0
#                     self.pos_pub.publish(msg)
#                 else:
#                     self.printPublishErrorCode("positioning", tag)
#                     self.setAnchorsManual()
#
#     def printPublishPosition(self, position, q, network_id):
#         if network_id is None:
#             network_id = 0
#         s = "ID:{}, x:{}, y:{}, z:{} , q:({},{},{},{})".format("0x%0.4x" % network_id, position.x, position.y, position.z,"%0.2f"%q.x,"%0.2f"%q.y,"%0.2f"%q.z,"%0.2f"%q.w)
#         print(s)
#
#     def setAnchorsManual(self):
#         for tag in self.tags:
#             status = self.pozyx.clearDevices(tag)
#             for anchor in self.anchors:
#                 status &= self.pozyx.addDevice(anchor, tag)
#             if len(self.anchors) > 4:
#                 status &= self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(self.anchors), remote_id=tag)
#             self.printPublishConfigurationResult(status, tag)
#
#     def printPublishConfigurationResult(self, status, tag_id):
#         if tag_id is None:
#             tag_id = 0
#         if status == POZYX_SUCCESS:
#             print("Configuration of tag %s: success" % tag_id)
#         else:
#             self.printPublishErrorCode("configuration", tag_id)
#
#     def printPublishErrorCode(self, operation, network_id):
#         error_code = SingleRegister()
#         status = self.pozyx.getErrorCode(error_code, None)
#         if network_id is None:
#             network_id = 0
#         if status == POZYX_SUCCESS:
#             print("Error %s on ID %s, %s" %
#                   (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
#         else:
#             # should only happen when not being able to communicate with a remote Pozyx.
#             self.pozyx.getErrorCode(error_code)
#             print("Error % s, local error code %s" % (operation, str(error_code)))
#
#     def printPublishAnchorConfiguration(self):
#         for anchor in self.anchors:
#             print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.pos)))
#
# if __name__ == "__main__":
#     try:
#         tms_ss_pozyx()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("tms_ss_pozyx node terminated.")

# #!/usr/bin/env python
# # -*- coding: utf-8 -*-
#
# from pypozyx import *
#
# class MultitagPositioning():
#     def __init__(self, pozyx, osc_udp_client, tags, anchors, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000, remote_id=None):
#         self.pozyx = pozyx
#         self.osc_udp_client = osc_udp_client
#
#         self.tags = tags
#         self.anchors = anchors
#         self.algorithm = algorithm
#         self.dimension = dimension
#         self.height = height
#         self.remote_id = remote_id
#
#     def setup(self):
#         """Sets up the Pozyx for positioning by calibrating its anchor list."""
#         print("------------POZYX MULTITAG POSITIONING V1.1 ------------")
#         print("NOTES:")
#         print("- Parameters required:")
#         print("\t- Anchors for calibration")
#         print("\t- Tags to work with")
#         print()
#         print("- System will manually calibration")
#         print()
#         print("System will auto start positioning")
#         print()
#         self.pozyx.printDeviceInfo(self.remote_id)
#         print()
#         print("------------POZYX MULTITAG POSITIONING V1.1 ------------")
#         print()
#
#
#         self.setAnchorsManual()
#         self.printPublishAnchorConfiguration()
#
#     def loop(self):
#         """Performs positioning and prints the results."""
#         for tag in self.tags:
#             position = Coordinates()
#             status = self.pozyx.doPositioning(
#                 position, self.dimension, self.height, self.algorithm, remote_id=tag)
#             if status == POZYX_SUCCESS:
#                 self.printPublishPosition(position, tag)
#             else:
#                 self.printPublishErrorCode("positioning", tag)
#
#     def printPublishPosition(self, position, network_id):
#         """Prints the Pozyx's position and possibly sends it as a OSC packet"""
#         if network_id is None:
#             network_id = 0
#         s = "POS ID: {}, x(mm): {}, y(mm): {}, z(mm): {}".format("0x%0.4x" % network_id, position.x, position.y, position.z)
#         print(s)
#         if self.osc_udp_client is not None:
#             self.osc_udp_client.send_message(
#                 "/position", [network_id, position.x, position.y, position.z])
#
#     def setAnchorsManual(self):
#         """Adds the manually measured anchors to the Pozyx's device list one for one."""
#         for tag in self.tags:
#             status = self.pozyx.clearDevices(tag)
#             for anchor in self.anchors:
#                 status &= self.pozyx.addDevice(anchor, tag)
#             if len(anchors) > 4:
#                 status &= self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(anchors), remote_id=tag)
#             # enable these if you want to save the configuration to the devices.
#             # self.pozyx.saveAnchorIds(tag)
#             # self.pozyx.saveRegisters([POZYX_ANCHOR_SEL_AUTO], tag)
#             self.printPublishConfigurationResult(status, tag)
#
#     def printPublishConfigurationResult(self, status, tag_id):
#         """Prints the configuration explicit result, prints and publishes error if one occurs"""
#         if tag_id is None:
#             tag_id = 0
#         if status == POZYX_SUCCESS:
#             print("Configuration of tag %s: success" % tag_id)
#         else:
#             self.printPublishErrorCode("configuration", tag_id)
#
#     def printPublishErrorCode(self, operation, network_id):
#         """Prints the Pozyx's error and possibly sends it as a OSC packet"""
#         error_code = SingleRegister()
#         status = self.pozyx.getErrorCode(error_code, self.remote_id)
#         if network_id is None:
#             network_id = 0
#         if status == POZYX_SUCCESS:
#             print("Error %s on ID %s, %s" %
#                   (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
#             if self.osc_udp_client is not None:
#                 self.osc_udp_client.send_message(
#                     "/error_%s" % operation, [network_id, error_code[0]])
#         else:
#             # should only happen when not being able to communicate with a remote Pozyx.
#             self.pozyx.getErrorCode(error_code)
#             print("Error % s, local error code %s" % (operation, str(error_code)))
#             if self.osc_udp_client is not None:
#                 self.osc_udp_client.send_message("/error_%s" % operation, [0, error_code[0]])
#
#     def printPublishAnchorConfiguration(self):
#         for anchor in self.anchors:
#             print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.pos)))
#             if self.osc_udp_client is not None:
#                 self.osc_udp_client.send_message(
#                     "/anchor", [anchor.network_id, anchor.pos.x, anchor.pos.y, anchor.pos.z])
#                 sleep(0.025)
#
#
# if __name__ == "__main__":
#     # shortcut to not have to find out the port yourself
#     serial_port = get_first_pozyx_serial_port()
#     if serial_port is None:
#         print("No Pozyx connected. Check your USB cable or your driver!")
#         quit()
#
#     remote_id = 0x1000                     # remote device network ID
#     remote = False                         # whether to use a remote device
#     if not remote:
#         remote_id = None
#
#     use_processing = False            # enable to send position data through OSC
#     ip = "127.0.0.1"                       # IP for the OSC UDP
#     network_port = 8888                    # network port for the OSC UDP
#     osc_udp_client = None
#
#
#     #tags = [0x6055]
#     tags = [0x6077]
#     # tags = [0x6055, 0x607a]        # remote tags
#     # necessary data for calibration
#     anchors = [DeviceCoordinates(0x605B, 1, Coordinates(4500,2884,995)),
#                             DeviceCoordinates(0x685D, 1, Coordinates(4805,146,2192)),
#                             DeviceCoordinates(0x603B, 1, Coordinates(8033,143,452)),
#                             DeviceCoordinates(0x683B, 1, Coordinates(11179,153,1370)),
#                             DeviceCoordinates(0x6822, 1, Coordinates(9778,5824,2533)),
#                             DeviceCoordinates(0x6031, 1, Coordinates(7531,5571,85)),
#                             DeviceCoordinates(0x680A, 1, Coordinates(6316,6215,1798)),
#                             DeviceCoordinates(0x6173, 1, Coordinates(7365,2246,3115))]
#
#     algorithm = POZYX_POS_ALG_UWB_ONLY     # positioning algorithm to use
#     dimension = POZYX_3D                   # positioning dimension
#     height = 1000                          # height of device, required in 2.5D positioning
#
#     pozyx = PozyxSerial(serial_port)
#     r = MultitagPositioning(pozyx, osc_udp_client, tags, anchors,
#                             algorithm, dimension, height, remote_id)
#     r.setup()
#     while True:
#         r.loop()
