#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from launch_mission.msg import ControlCommand  # Import the custom message

# Global variable to store the robot's state (start or stop)
robot_state = "stop"

def control_command_callback(data):
    global robot_state
    if data.command == "start":
        rospy.loginfo("Starting the robot.")
        robot_state = "start"
    elif data.command == "stop":
        rospy.loginfo("Stopping the robot.")
        robot_state = "stop"

def control_robot():
    rospy.init_node('velocity_commander', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    # Subscribe to the control command topic
    rospy.Subscriber('/control_command', ControlCommand, control_command_callback)

    while not rospy.is_shutdown():
        cmd = Twist()

        # Check the robot's state and set velocities accordingly
        if robot_state == "start":
            cmd.linear.x = 0.1  # Set linear velocity (m/s)
            cmd.angular.z = 0.0  # Set angular velocity (rad/s)
        elif robot_state == "stop":
            cmd.linear.x = 0.0  # Set linear velocity to zero (m/s)
            cmd.angular.z = 0.0  # Set angular velocity to zero (rad/s)

        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        control_robot()
    except rospy.ROSInterruptException:
        pass
