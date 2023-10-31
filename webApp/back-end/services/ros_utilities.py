import time
import roslibpy
from roslibpy import Topic, Ros
from flask import jsonify

ros_connections = {}
def create_topic(ros, topic_name):
    sound_topic = Topic(ros, topic_name, 'std_msgs/String')
    sound_topic.advertise()
    return sound_topic

def launch_mission(robot_ip, command):
    try:
        if robot_ip in ros_connections:
            ros = ros_connections[robot_ip]
        else:
            ros = Ros(robot_ip, 9090)
            ros.run()
            ros_connections[robot_ip] = ros
        # code only to test functionality, will be replaced ...
        create_topic(ros, '/launch_command').publish({'data': command})
        time.sleep(7)
        control_command_topic = Topic(ros, '/control_command', 'launch_mission/ControlCommand')
        control_command_topic.publish({'command': "start"})
        time.sleep(5)
        control_command_topic.publish({'command': 'stop'})

        return jsonify({"message": "Command sent successfully"})
    
    except Exception as e:
        return jsonify({"error": str(e)}), 500  

def send_command(robot_ip, command):
    try:
        if robot_ip in ros_connections:
            ros = ros_connections[robot_ip]
        else:
            ros = Ros(robot_ip, 9090)
            ros.run()
            ros_connections[robot_ip] = ros

        create_topic(ros, '/launch_command').publish({'data': command})
        return jsonify({"message": "Command sent successfully"})
    
    except Exception as e:
        return jsonify({"error": str(e)}), 500  