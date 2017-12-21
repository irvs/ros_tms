#!/usr/bin/env python

import rospy
from time import sleep
from pypozyx import *
from std_msgs.msg import UInt16MultiArray, MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import PoseStamped

class LocalizeClass(object):

    def __init__(self):
        rospy.init_node('pozyx_localize', anonymous=True)
        self.tag_pub = rospy.Publisher("pozyx_rawdata", PoseStamped, queue_size=1000)
        self.anchors_pub = rospy.Publisher("positioning_anchors_id", UInt16MultiArray, queue_size=1000)

        serial_port = get_first_pozyx_serial_port()
        if serial_port is None:
            print("No Pozyx Connected.")
            quit()

        ########## PARAMETERS ##########

        remote_id = 0x6069
        remote = False
        if not remote:
            remote_id = None

        ########## Anchors (network_id, flag, pos) ##########

        anchors = [DeviceCoordinates(0x6023, 1, Coordinates(-2479, -16692, 0)),
                   DeviceCoordinates(0x6037, 1, Coordinates(-6131, -4699, 0)),
                   DeviceCoordinates(0x6044, 1, Coordinates(-17374, -8100, 0)),
                   DeviceCoordinates(0x6050, 1, Coordinates(-13705, -20161, 0))]

        ################################

        self.pozyx = PozyxSerial(serial_port)
        self.remote_id = remote_id
        self.remote = remote
        self.anchors = anchors
        self.algorithm = POZYX_POS_ALG_UWB_ONLY
        self.dimension = POZYX_2D
        self.height = 1000
        self.frame_id = "world_link"

        self.setup()

        while not rospy.is_shutdown():
            try:
                self.loop()

            except: continue

    def setup(self):

        self.setAnchors()
        self.printCalibrationResult()

        sleep(1.0)
        print("----------UWB Info----------")
        print("-----Anchors-----")
        for anchor in self.anchors:
            self.printUWBInfo(anchor.network_id)
        if self.remote:
            print("-----Remote Tag-----")
            self.printUWBInfo(self.remote_id)
            print("-----Local Tag-----")
            self.printUWBInfo(None)
        else:
            print("-----Tag-----")
            self.printUWBInfo(self.remote_id)
        print("----------------------------")
        sleep(0.5);

    def loop(self):

        position = Coordinates()
        orientation = Quaternion()

        status = self.pozyx.doPositioning(
            position, self.dimension, self.height, self.algorithm, self.remote_id)

        self.pozyx.getQuaternion(orientation, self.remote_id)

        if status == POZYX_SUCCESS:
            self.printPositioningData(position, orientation)
            self.printPositioningAnchors()
        else:
            print("Positioning Error")

    def printPositioningData(self, position, orientation):

        pozyx_pose = PoseStamped()
        pozyx_pose.header.frame_id = self.frame_id;
        pozyx_pose.header.stamp = rospy.Time.now()
        pozyx_pose.pose.position.x = position.x * 0.001
        pozyx_pose.pose.position.y = position.y * 0.001
        pozyx_pose.pose.position.z = position.z * 0.001
        pozyx_pose.pose.orientation.x = orientation.x
        pozyx_pose.pose.orientation.y = orientation.y
        pozyx_pose.pose.orientation.z = orientation.z
        pozyx_pose.pose.orientation.w = orientation.w

        self.tag_pub.publish(pozyx_pose)

    def printPositioningAnchors(self):

        num_positioning_anchors = SingleRegister()
        self.pozyx.getNumberOfAnchors(num_positioning_anchors, self.remote_id)

        positioning_anchors = DeviceList(list_size=num_positioning_anchors[0])
        self.pozyx.getPositioningAnchorIds(positioning_anchors, self.remote_id)

        arr = []
        for i in range(num_positioning_anchors[0]):
            arr.append(positioning_anchors[i])

        self.anchors_pub.publish(UInt16MultiArray(data=arr))

    def setAnchors(self):

        self.pozyx.clearDevices(self.remote_id)
        for anchor in self.anchors:
            self.pozyx.addDevice(anchor, self.remote_id)
        # if len(self.anchors) > 4:
        #     status &= self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(self.anchors))

    def printCalibrationResult(self):

        list_size = SingleRegister()

        self.pozyx.getDeviceListSize(list_size, self.remote_id)
        if list_size[0] != len(self.anchors):
            print("Configuration Error")
            return
        device_list = DeviceList(list_size=list_size[0])
        self.pozyx.getDeviceIds(device_list, self.remote_id)
        print("----------Calibration result----------")
        print("Anchors found: {0}".format(list_size[0]))
        print("Anchor IDs: ")

        for i in range(list_size[0]):
            anchor_coordinates = Coordinates()
            status = self.pozyx.getDeviceCoordinates(
                device_list[i], anchor_coordinates, self.remote_id)
            print("ANCHOR, 0x%0.4x, %s" % (device_list[i], str(anchor_coordinates)))

        print("--------------------------------------")

    def printUWBInfo(self, device_id):
        uwb_info = UWBSettings()

        self.pozyx.getUWBSettings(uwb_info, device_id)

        if device_id is None:
            print("ID: 0x0000")
        else:
            print("ID: 0x%0.4x" % device_id)

        print("Channel: {info.channel}".format(info=uwb_info))
        print("Gain: {info.gain_db}".format(info=uwb_info))
        print(' ')

if __name__ == "__main__":
    try:
        LocalizeClass()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("pozyx_localize node finished.")
