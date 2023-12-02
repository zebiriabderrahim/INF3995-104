#!/usr/bin/env python3
import rospy
import subprocess
import threading
from std_msgs.msg import String
import os

def launch_command_in_thread(launch_command):
    try:
        rospy.loginfo(f"Received launch command: {launch_command}")
        rospy.loginfo("Executing command in a new terminal: " + launch_command)
        subprocess.call(launch_command, shell=True)
    except Exception as e:
        rospy.logerr(f"Error executing launch command: {str(e)}")

def launch_callback(msg):
    rospy.loginfo(f"data command: {msg.data}")
    thread = threading.Thread(target=launch_command_in_thread, args=(msg.data,))
    thread.start()

def main():
    rospy.init_node("launch_executor")
    rospy.Subscriber("/launch_command", String, launch_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
