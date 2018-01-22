#!/usr/bin/env python
# coding:utf-8

import rospy
import sys
from time import sleep
from pypozyx import *
from std_msgs.msg import UInt16MultiArray, MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import PoseStamped

class IdWithDistance(object):
    def __init__(self, _network_id, _distance):
        self.network_id = _network_id
        self.distance = _distance

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

        distance_threshold = 40.0 # [m]
        ranging_repetition = 10
        ranging_interval = 1000

        nr_anchors = 4

        ########## Anchors (network_id, flag, pos) ##########

        anchors = [DeviceCoordinates(0x6e30, 1, Coordinates(-63422, -22248, 0)),
                   DeviceCoordinates(0x6e39, 1, Coordinates(-51793, -18700, 0)),
                   DeviceCoordinates(0x6e22, 1, Coordinates(-34670, -13403, 0)),
                   DeviceCoordinates(0x6e31, 1, Coordinates(-28827, -11650, 0)),
                   DeviceCoordinates(0x6044, 1, Coordinates(-17374, -8100, 0)),
                   DeviceCoordinates(0x6037, 1, Coordinates(-6131, -4699, 0)),
                   DeviceCoordinates(0x6e49, 1, Coordinates(-59680, -34253, 0)),
                   DeviceCoordinates(0x6e23, 1, Coordinates(-48108, -30702, 0)),
                   DeviceCoordinates(0x6e08, 1, Coordinates(-36752, -27245, 0)),
                   DeviceCoordinates(0x6e58, 1, Coordinates(-25118, -23650, 0)),
                   DeviceCoordinates(0x6050, 1, Coordinates(-13705, -20161, 0)),
                   DeviceCoordinates(0x6023, 1, Coordinates(-2479, -16692, 0))]

        ################################

        self.pozyx = PozyxSerial(serial_port)
        self.remote_id = remote_id
        self.remote = remote
        self.distance_threshold = distance_threshold * 1000
        self.ranging_repetition = ranging_repetition
        self.counter = 0
        self.ranging_interval = ranging_interval
        self.anchors = anchors
        self.nr_anchors = nr_anchors
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
        sleep(0.5)

    def loop(self):

        self.counter %= self.ranging_interval

        if(self.counter == 0):
            self.selectPositioningAnchors()

        position = Coordinates()
        orientation = Quaternion()

        status = self.pozyx.doPositioning(
            position, self.dimension, self.height, self.algorithm, self.remote_id)

        self.pozyx.getQuaternion(orientation, self.remote_id)

        if status == POZYX_SUCCESS:
            self.printPositioningData(position, orientation)
        else:
            print("Positioning Error")

        self.counter += 1

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

    def selectPositioningAnchors(self):

        measured_distances = []
        for anchor in self.anchors:
            distances_of_an_anchor = []
            for i in range(self.ranging_repetition):
                range_data = DeviceRange()
                self.pozyx.doRanging(anchor.network_id, range_data, self.remote_id)
                if(range_data.distance != 0 and range_data.distance <= self.distance_threshold):
                    distances_of_an_anchor.append(range_data.distance)
            measured_distances.append(distances_of_an_anchor)

        num_valid_anchors = 0
        distances_average = []

        for obj in measured_distances:
            if len(obj) == 0:
                distances_average.append(sys.maxint)
            else:
                distances_average.append(sum(obj)/len(obj))
                num_valid_anchors += 1

        if num_valid_anchors < self.nr_anchors:
            return

        candidate_anchors = []
        for i in range(len(self.anchors)):
            data = IdWithDistance(self.anchors[i].network_id, distances_average[i])
            candidate_anchors.append(data)
        candidate_anchors.sort(key=lambda x: x.distance)

        selected_anchors = DeviceList(list_size=self.nr_anchors)
        arr = []
        for j in range(self.nr_anchors):
            selected_anchors[j] = candidate_anchors[j].network_id
            arr.append(selected_anchors[j])
        self.pozyx.setPositioningAnchorIds(selected_anchors, self.remote_id)

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
