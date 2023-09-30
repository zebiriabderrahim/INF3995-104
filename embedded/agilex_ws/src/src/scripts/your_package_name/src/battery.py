#!/usr/bin/env python

import rospy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

limo=limo.LIMO()
limo.EnableCommand()
def handle_battery_state(data):
  rospy.loginfo(f"Received launch command: {data}")
  rospy.loginfo("yes")
  # Get the robot's battery percentage.
  robot_battery_percentage = data

  # Publish the robot's battery percentage to the topic that the backend is subscribing to.
  publisher.publish(robot_battery_percentage)

# Connect to the ROS master.
rospy.init_node('battery_life_publisher')

# Create a ROS subscriber to subscribe to the `/battery_state` topic.
subscriber = rospy.Subscriber('/battery_state', BatteryState, handle_battery_state)

# Create a ROS publisher to publish the robot's battery percentage to the topic that the backend is subscribing to.
publisher = rospy.Publisher('/robot_battery_percentage', Float32, queue_size=10)

#
timer = rospy.Timer(rospy.Duration(1.0), handle_battery_state)

try:
  rospy.spin()
except KeyboardInterrupt:
  rospy.signal_shutdown()
