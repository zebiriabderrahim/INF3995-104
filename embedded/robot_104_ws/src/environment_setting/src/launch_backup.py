#!/usr/bin/env python

import rospy
import subprocess
from std_msgs.msg import String

def launch_callback(msg):
    try:
        launch_command = msg.data
        rospy.loginfo(f"Received launch command: {launch_command.split()}")
        command_to_execute = [
            "roslaunch",
            f"{launch_command}"
        ]
        rospy.loginfo("Executing command: " + " ".join(command_to_execute))
        subprocess.Popen(command_to_execute)

    except Exception as e:
        rospy.logerr(f"Error executing launch command: {str(e)}")

def main():
    rospy.init_node("launch_executor")
    rospy.Subscriber("/launch_command", String, launch_callback)
    rospy.spin()

if __name__ == "__main__":
    main()

