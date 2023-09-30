#!/usr/bin/env python3
# coding=UTF-8

import rospy
from std_msgs.msg import Float32
from pylimo import limo
import time

def voltage_to_percentage(voltage):
    # Calculate battery percentage based on the voltage range
    min_voltage = 8.25
    max_voltage = 12.3

    percentage = (voltage - min_voltage) / (max_voltage - min_voltage) * 100.0

    # Ensure the percentage is within the valid range (0-100)
    percentage = max(0.0, min(100.0, percentage))

    return percentage

def publish_battery_percentage():
    # Initialize the ROS node
    rospy.init_node('battery_percentage_publisher', anonymous=True)
    
    # Create a publisher for the battery percentage topic
    battery_percentage_pub = rospy.Publisher('/battery_percentage', Float32, queue_size=10)

    # Initialize the LIMO object
    limo = limo.LIMO()
    limo.EnableCommand()  # Enable control

    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
        battery_voltage = limo.GetBatteryVoltage()
        battery_percentage = voltage_to_percentage(battery_voltage)
        
        rospy.loginfo("Battery Percentage: %f%%", battery_percentage)

        # Publish the battery percentage to the '/battery_percentage' topic
        battery_percentage_pub.publish(battery_percentage)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_battery_percentage()
    except rospy.ROSInterruptException:
        pass

