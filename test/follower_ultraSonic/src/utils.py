#!/usr/bin/python
#-*-coding:utf-8-*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray


def CUBE():
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 1
    # set marker.pose
    # set lifetime marker.lifetime = ros::Duration();
    return marker


def ARROW():
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 1
    # set marker.pose
    # set lifetime marker.lifetime = ros::Duration();
    return marker


def TEXT():
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.type = Marker.TEXT_VIEW_FACING
    marker.pose.orientation.w = 1.0
    marker.action = Marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    marker.color.a = 1
    # set marker.pose
    # set lifetime marker.lifetime = ros::Duration();
    return marker