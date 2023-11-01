import time
from roslibpy import Topic, Ros
from flask import jsonify
from services import socket_manager
from functools import partial
import threading

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
            print("ros subscribe", robot_ip)

            print("ros subscribe after")

            # ros_thread = threading.Thread(target=ros.run)
            # ros_thread.daemon = True
            # ros_thread.start()
            print("after ros run forve")
        create_topic(ros, '/launch_command').publish({'data': command})
        return jsonify({"message": "Command sent successfully"})
    
    except Exception as e:
        return jsonify({"error": str(e)}), 500  

def subscribe_to_battery(robot_ip):
    try:
        if robot_ip in ros_connections:
            ros = ros_connections[robot_ip]
        else:
            ros = Ros(robot_ip, 9090)
            ros.run()
            ros_connections[robot_ip] = ros

        battery_topic = Topic(ros, '/battery_percentage', 'std_msgs/Float32')
        battery_topic.subscribe(partial(socket_manager.send_robot_battery, robot_ip))
        return jsonify({"message": "Command sent successfully"})
    
    except Exception as e:
        return jsonify({"error": str(e)}), 500