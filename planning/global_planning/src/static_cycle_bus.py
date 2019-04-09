#!/usr/bin/python2
# -*-coding:utf-8-*-

import rospy
from smartcar_msgs.msg import Lane, Waypoint
from std_msgs.msg import String
import csv
import os


class APP(object):
    def __init__(self):
        rospy.init_node("static_cycle_bus")
        self.index = 0
        # self.lanes = []

    def run(self):
        path_file = rospy.get_param("~cyclebus_pathfile", None)
        self.lanes = self.load_path(path_file)
        self.pub_path = rospy.Publisher("/global_path", Lane, queue_size=1)
        rospy.Subscriber("/request", String, callback=self.request_handler, queue_size=1)

        rospy.spin()

    def load_trajectory(self, file_name):
        trajectory = Lane()
        trajectory.header.frame_id = "map"
        with open(file_name, 'r') as csvfile:
            csv_reader = csv.reader(csvfile)
            for row in csv_reader:
                waypoint = Waypoint()
                waypoint.speed_limit = 3.0
                waypoint.pose.header.frame_id = "map"
                waypoint.pose.pose.position.x = float(row[0])
                waypoint.pose.pose.position.y = float(row[1])
                waypoint.pose.pose.position.z = float(row[2])
                waypoint.pose.pose.orientation.x = float(row[3])
                waypoint.pose.pose.orientation.y = float(row[4])
                waypoint.pose.pose.orientation.z = float(row[5])
                waypoint.pose.pose.orientation.w = float(row[6])
                trajectory.waypoints.append(waypoint)
        return trajectory

    def load_path(self, file_path):
        if file_path is None:
            print("cyclebus_pathfile seems doesn't exist, please confirm.")
            exit(1)
        if not os.path.isdir(file_path):
            print file_path + " is not dir, please confirm"
        files = os.listdir(file_path)
        files.sort(key=lambda x: x[0])
        trajectory_msgs = []
        for file in files:
            trajectory = self.load_trajectory(file_path + file)
            trajectory_msgs.append(trajectory)
        return trajectory_msgs

    def request_handler(self, mode):
        if mode.data == "request cycle bus path":
            print("cycle_bus plan: received request.")
            print("cycle_bus plan: publish path-" + str(self.index))
            self.pub_path.publish(self.lanes[self.index])
            self.index = (self.index + 1) % len(self.lanes)
        else:
            return


if __name__ == '__main__':
    try:
        app = APP()
        app.run()
    except KeyboardInterrupt:
        print("Shutting down cycle_bus node")
        exit(0)
    except Exception as e:
        print e
        exit(1)
