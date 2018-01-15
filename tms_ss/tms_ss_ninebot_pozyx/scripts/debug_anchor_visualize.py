#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

def publish_anchors():

    pub_anchors = rospy.Publisher('/pozyx_anchors_marker', MarkerArray, queue_size=10)
    markerarray = MarkerArray()

    # 座標変換, Offset適用後

    transformed_x = [10844+6966, -6966+6966, -23361+6966, -19237+6966, -19430+6966,-27795+6966, -28156+6966, -22869+6966]
    transformed_y = [-(0+42), -(-42+42), -(3805+42), -(-17218+42), -(-32293+42), -(-40438+42), -(-48587+42), -(-55671+42)]

    for i in range(8):
        marker = Marker()
        marker.header.frame_id = "/world_link"
        marker.header.stamp    = rospy.Time.now()
        marker.id              = i + 10
        marker.type            = 3 # cylinder
        marker.action          = 0 # add
        marker.scale.x         = 0.2
        marker.scale.y         = 0.2
        marker.scale.z         = 1.0
        marker.color.r         = 1.0
        marker.color.g         = 0.0
        marker.color.b         = 1.0
        marker.color.a         = 1.0
        marker.lifetime = rospy.Duration()
        marker.pose.position.x = transformed_x[i] * 0.001
        marker.pose.position.y = transformed_y[i] * 0.001
        markerarray.markers.append(marker)

    pub_anchors.publish(markerarray)


rospy.init_node('anchor_visualize', anonymous=True)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
	    publish_anchors()
    except rospy.ROSInterruptException:
        rospy.loginfo("debug_marker_generate terminated")
