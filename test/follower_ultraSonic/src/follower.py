#!/usr/bin/python
# -*-coding:utf-8-*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from smartcar_msgs.msg import DiffSonic
import utils
from can_msgs.msg import ecu
import numpy as np


class APP():
    def __init__(self):
        rospy.init_node("sonic_follower_node")

    def _initial_params(self):
        self.sonic_topic = rospy.get_param("~sonic_topic", "/sonic_raw")
        self.pub_topic = rospy.get_param("~pub_topic", "/ecu")
        self.receiver_width = rospy.get_param("~receiver_width", 1.0)
        self.target_cnt = 0
        self.if_vis = rospy.get_param("~visualize", False)

    def _add_sub_(self):
        rospy.Subscriber(self.sonic_topic, DiffSonic, self._sonic_handler, queue_size=1)

    def _add_pub_(self):
        self.pub_cmd = rospy.Publisher(self.pub_topic, ecu, queue_size=1)
        self.pub_vis = rospy.Publisher("/follower_vis", MarkerArray, queue_size=10)

    def _is_validate(self, dist_left, dist_right):
        if dist_left > 4.0 or dist_right > 4.0:
            self.target_cnt += 1
            if self.target_cnt > 50:
                print("sonic_follower : No target detected !")
                self.target_cnt = 0
            return False
        elif (dist_left + dist_right) <= self.receiver_width or (dist_left + self.receiver_width) <= dist_right or (
                dist_right + self.receiver_width) <= dist_left:
            print("Sonic follower : Fatal Error: Cannot make Triangle")
            print("--> width:{} left:{} right:{}".format(self.receiver_width, dist_left, dist_right))
            return False
        self.target_cnt = 0
        return True

    def _sonic_handler(self, msg):
        if msg.type != "sonic_good":
            return
        dist_left = msg.left / 1000.0
        dist_right = msg.right / 1000.0
        if not self._is_validate(dist_left, dist_right):
            return
        dist, angle = self._get_dis_angle(self.receiver_width, dist_left, dist_right)
        # print angle
        msg_ctrl = self._get_cmd_ctrl(dist, angle)
        self.pub_cmd.publish(msg_ctrl)
        if self.if_vis:
            self._visualize_(dist, angle)

    def _get_dis_angle(self, width, l_left, l_right):
        width = float(width)
        l_left = float(l_left)
        l_right = float(l_right)

        p = (width + l_left + l_right) / 2
        S = np.sqrt(p * (p - width) * (p - l_left) * (p - l_right))
        dis = 2 * S / width
        l_midline = np.sqrt(l_left * l_left / 2 + l_right * l_right / 2 - width * width / 4)
        angle = np.arccos(dis / l_midline)
        if l_left < l_right:
            return dis, -angle
        else:
            return dis, angle

    def _get_cmd_ctrl(self, dist, angle):
        msg_ecu = ecu()
        if dist < 1.0:
            msg_ecu.motor = 0.0
            msg_ecu.shift = ecu().SHIFT_N
            return msg_ecu
        msg_ecu.motor = 2.0
        if angle > 0:
            msg_ecu.steer = angle / 0.06 * 140
        else:
            msg_ecu.steer = angle / 0.06 * 93
        msg_ecu.shift = ecu().SHIFT_D
        return msg_ecu

    def _visualize_(self, dist, angle):
        marker_left = utils.CUBE()
        marker_right = utils.CUBE()
        msg_vis = MarkerArray()

        marker_left.header.frame_id = "base_link"
        marker_left.id = 0
        marker_left.scale.x = 0.1
        marker_left.scale.y = 0.1
        marker_left.scale.z = 0.1
        marker_left.pose.position.x = 0
        marker_left.pose.position.y = self.receiver_width / 2
        msg_vis.markers.append(marker_left)

        text_left = utils.TEXT()
        text_left.header.frame_id = "base_link"
        text_left.id = 1
        text_left.text = "left"
        text_left.pose.position.x = -1.0
        text_left.pose.position.y = self.receiver_width / 2
        text_left.scale.x = text_left.scale.y = text_left.scale.z = 0.3
        msg_vis.markers.append(text_left)

        marker_right.header.frame_id = "base_link"
        marker_right.id = 2
        marker_right.scale.x = 0.1
        marker_right.scale.y = 0.1
        marker_right.scale.z = 0.1
        marker_right.pose.position.x = 0
        marker_right.pose.position.y = -self.receiver_width / 2
        msg_vis.markers.append(marker_right)

        text_right = utils.TEXT()
        text_right.header.frame_id = "base_link"
        text_right.id = 3
        text_right.text = "right"
        text_right.pose.position.x = -1.0
        text_right.pose.position.y = -self.receiver_width / 2
        text_right.scale.x = text_right.scale.y = text_right.scale.z = 0.3
        msg_vis.markers.append(text_right)

        marker_target = utils.ARROW()
        marker_target.header.frame_id = "base_link"
        marker_target.id = 4
        marker_target.pose.position.x = dist * np.cos(angle)
        marker_target.pose.position.y = -dist * np.sin(angle)
        marker_target.pose.position.z = 0
        marker_target.scale.x = 0.3
        marker_target.scale.y = marker_target.scale.z = 0.1
        # TODO::set orientation of target
        msg_vis.markers.append(marker_target)

        text = utils.TEXT()
        text.id = 5
        text.header.frame_id = "base_link"
        text.text = " dist: {} \n angle: {}".format(dist, angle)
        text.pose.position.x = 4.0
        text.pose.position.y = 4.0
        text.scale.x = text.scale.y=text.scale.z = 0.3
        msg_vis.markers.append(text)

        self.pub_vis.publish(msg_vis)

    def run(self):
        self._initial_params()
        self._add_pub_()
        self._add_sub_()
        rospy.spin()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        return self


if __name__ == '__main__':
    try:
        app = APP()
        app.run()
    except Exception as e:
        print e
