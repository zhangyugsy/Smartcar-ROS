#!/usr/bin/python
# -*-coding:utf-8-*-

import rospy
import smartcar_msgs
import numpy as np


# 前面使用5-10次观测值的平均数,作为输入left right
def get_dis_angle(width, l_left, l_right):
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


print get_dis_angle(3, 4, 5)