#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate

class MapRepublisher:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        
        self.subscriber_odom = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.subscriber_map = rospy.Subscriber("/map", OccupancyGrid, self.callback_map)
        self.subscriber_updates = rospy.Subscriber("/map_updates", OccupancyGridUpdate, self.callback_updates)
        
        self.publisher_odom = rospy.Publisher("/{}/odom".format(robot_name), Odometry, queue_size=10)
        self.publisher_map = rospy.Publisher("/{}/map".format(robot_name), OccupancyGrid, queue_size=10)
        self.publisher_updates = rospy.Publisher("/{}/map_updates".format(robot_name), OccupancyGridUpdate, queue_size=10)

    def callback_map(self, data):
        self.publisher_map.publish(data)

    def callback_odom(self, data):
        self.publisher_odom.publish(data)

    def callback_updates(self, data):
        self.publisher_updates.publish(data)

if __name__ == '__main__':
    try:
        rospy.init_node('map_republisher')
        robot_name = sys.argv[1] # Retrieve the robot name from command line arguments
        republisher = MapRepublisher(robot_name)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass