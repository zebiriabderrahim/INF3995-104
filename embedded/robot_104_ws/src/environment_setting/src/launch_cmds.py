#!/usr/bin/env python

import rospy
import subprocess
from std_msgs.msg import String

def source_setup_script():
    script_path = '/home/nvidia/INF3995-104/embedded/robot_104_ws/setup_bash.sh'  # Replace with the actual path to your script
    # Execute the script using subprocess
    subprocess.call(['bash', script_path])

def launch_callback(msg):
    try:
        #source_setup_script()
        launch_command = msg.data
        rospy.loginfo(f"Received launch command: {launch_command.split()}")
        command_to_execute = [
            "roslaunch", f"/home/nvidia/INF3995-104/embedded/robot_104_ws/src/{launch_command}"]
        rospy.loginfo("Executing command: " + " ".join(command_to_execute))
        subprocess.Popen(command_to_execute)

    except Exception as e:
        rospy.logerr(f"Error executing launch command: {str(e)}")

def main():
    rospy.init_node("launch_executor")
    rospy.loginfo(f"Received launch command")

    rospy.Subscriber("/launch_command", String, launch_callback)
    rospy.spin()

if __name__ == "__main__":
    main()

