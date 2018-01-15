#!/usr/bin/env python
# coding: UTF-8

import rospy
from pypozyx import *
from std_msgs.msg import UInt16MultiArray, MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class VisualizeClass(object):

    def __init__(self):
        rospy.init_node('pozyx_visualize', anonymous=True)
        self.tag_pub = rospy.Publisher("pozyx_tag_marker", Marker, queue_size=1000)
        self.anchors_pub = rospy.Publisher("pozyx_anchors_marker", MarkerArray, queue_size=1000)

        rospy.Subscriber("/pozyx", PoseStamped, self.callback_tag, queue_size=1000)
        rospy.Subscriber("/positioning_anchors_id", UInt16MultiArray, self.callback_anchors, queue_size=1000)

        ########## Anchors (network_id, flag, pos) ###########

        anchors = [DeviceCoordinates(0x6e31, 1, Coordinates(10844+6966, -(0+42), 0)),
           DeviceCoordinates(0x6e49, 1, Coordinates(-6966+6966, -(-42+42), 0)),
           DeviceCoordinates(0x6e08, 1, Coordinates(-23361+6966, -(3805+42), 0)),
           DeviceCoordinates(0x6050, 1, Coordinates(-19237+6966, -(-17218+42), 0))]

        # anchors = [DeviceCoordinates(0x6044, 1, Coordinates(-19430+6966, -(-32293+42), 0)),
        #             DeviceCoordinates(0x6e22, 1, Coordinates(-27795+6966, -(-40438+42), 0)),
        #             DeviceCoordinates(0x6e30, 1, Coordinates(-28156+6966, -(-48587+42), 0)),
        #             DeviceCoordinates(0x6037, 1, Coordinates(-22869+6966, -(-55671+42), 0))]

        ################################

        self.anchors = anchors
        self.frame_id = "world_link"
        self.isNinebot = False
        self.isHuman = True

        rospy.spin()

    def callback_tag(self, data):

        marker_tag = Marker()
        marker_tag.header.frame_id = self.frame_id
        marker_tag.header.stamp = rospy.Time.now()
        marker_tag.id = 0
        marker_tag.action = Marker().ADD

        if self.isNinebot:
            marker_tag.type = Marker().MESH_RESOURCE
            marker_tag.mesh_resource = "package://tms_ss_ninebot_pozyx/meshes/ninebot_v2.dae"
            marker_tag.mesh_use_embedded_materials = True
            marker_tag.scale.x = 0.01
            marker_tag.scale.y = 0.01
            marker_tag.scale.z = 0.01
        elif self.isHuman:
            marker_tag.type = Marker().MESH_RESOURCE
            marker_tag.mesh_resource = "package://tms_ss_ninebot_pozyx/meshes/WalkingMan4.dae"
            marker_tag.mesh_use_embedded_materials = True
            marker_tag.scale.x = 0.025
            marker_tag.scale.y = 0.025
            marker_tag.scale.z = 0.025
        else:
            marker_tag.type = Marker().CYLINDER
            marker_tag.scale.x = 0.2
            marker_tag.scale.y = 0.2
            marker_tag.scale.z = 1.0
            marker_tag.color.r = 1.0
            marker_tag.color.g = 0.0
            marker_tag.color.b = 0.0
            marker_tag.color.a = 1.0

        marker_tag.pose = data.pose
        marker_tag.lifetime = rospy.Duration()

        self.tag_pub.publish(marker_tag)

    def callback_anchors(self, data):

        markerArray_anchors = MarkerArray()

        for anchor in self.anchors:

            marker_anchor = Marker()
            marker_anchor.header.frame_id = self.frame_id
            marker_anchor.header.stamp = rospy.Time.now()
            marker_anchor.id = anchor.network_id
            marker_anchor.action = Marker().ADD
            marker_anchor.type = Marker().CYLINDER
            marker_anchor.scale.x = 0.2
            marker_anchor.scale.y = 0.2
            marker_anchor.scale.z = 1.0
            marker_anchor.color.a = 1.0
            marker_anchor.lifetime = rospy.Duration()
            marker_anchor.pose.position.x = anchor.pos.x * 0.001
            marker_anchor.pose.position.y = anchor.pos.y * 0.001
            marker_anchor.pose.position.z = 0.5

            for i in range(len(data.data)):
                if marker_anchor.id == data.data[i]:
                    marker_anchor.color.r = 0.0
                    marker_anchor.color.g = 1.0
                    marker_anchor.color.b = 0.0
                    break
                else:
                    marker_anchor.color.r = 0.8
                    marker_anchor.color.g = 0.8
                    marker_anchor.color.b = 0.8

            markerArray_anchors.markers.append(marker_anchor)

        self.anchors_pub.publish(markerArray_anchors)

if __name__ == "__main__":
    try:
        VisualizeClass()
    except rospy.ROSInterruptException:
        rospy.loginfo("pozyx_localize node finished.")
