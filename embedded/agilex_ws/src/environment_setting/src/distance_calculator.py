#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from math import sqrt

class OdomDistanceCalculator:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.loginfo("Subscribed to /odom topic.")
        self.total_distance = 0.0
        self.last_x, self.last_y, self.last_z = 0.0, 0.0, 0.0
        self.first_measurement = True

        self.distance_pub = rospy.Publisher('distance', Float64, queue_size=10)

    def odom_callback(self, msg):
        x, y, z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        rospy.logdebug("Received Odometry: x = %f, y = %f, z = %f", x, y, z)

        if self.first_measurement:
            rospy.logdebug("Initializing first measurement.")
            self.last_x, self.last_y, self.last_z = x, y, z
            self.first_measurement = False
        else:
            distance_increment = sqrt((x - self.last_x)**2 + (y - self.last_y)**2 + (z - self.last_z)**2)
            self.total_distance += distance_increment
            rospy.logdebug("Distance increment: %f meters, Total distance: %f meters", distance_increment, self.total_distance)
            self.last_x, self.last_y, self.last_z = x, y, z

    def publish_distance(self):
        distance_msg = Float64()
        distance_msg.data = self.total_distance
        self.distance_pub.publish(distance_msg)

if __name__ == '__main__':
    rospy.init_node('odom_distance_calculator')
    calculator = OdomDistanceCalculator()

    rate = rospy.Rate(1)  

    while not rospy.is_shutdown():
        calculator.publish_distance()
        rate.sleep()

