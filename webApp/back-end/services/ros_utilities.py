import time
from roslibpy import Topic, Ros
from flask import jsonify
from services import socket_manager
from functools import partial

ros_connections = {}
robots_ip=[]

def launch_mission(robot_ip, command):
    try:
        ros = create_ros(robot_ip)
        execute_command(robot_ip, command)
        time.sleep(7)
        control_command_topic = Topic(
            ros, '/control_command', 'std_msgs/String')
        control_command_topic.publish({'command': "start"})
        time.sleep(5)
        control_command_topic.publish({'command': "stop"})
        return jsonify({"message": "Command sent successfully"})

    except Exception as e:
        return jsonify({"error": str(e)}), 500
    
def terminate_mission(robot):
    print("terminate mission")
    try:
        ros = create_ros(robot["ipAddress"])
        robot_name = str(robot["name"]).replace(" ", "").lower()
        odom_topic = Topic(ros, f'/{robot_name}/odom', 'nav_msgs/Odometry')
        map_topic = Topic(ros, f'/{robot_name}/map', 'nav_msgs/Odometry')
        execute_command(robot["ipAddress"], f'rosnode kill /{robot_name}/explore')
        time.sleep(0.4)
        
        odom_topic.unsubscribe()
        map_topic.unsubscribe()
        
    except Exception as e:
        return jsonify({"error": str(e)}), 500

def subscribe_to_battery(robot_ip):
    try:
        if robot_ip not in ros_connections:
            ros = create_ros(robot_ip)
            execute_command(robot_ip, 'rosrun battery battery.py')
        
            Topic(ros, '/battery_percentage', 'std_msgs/Float32').subscribe(
                partial(socket_manager.send_robot_battery, robot_ip))
            
        return jsonify({"message": "Command sent successfully"})

    except Exception as e:
        return jsonify({"error": str(e)}), 500


def execute_command(robot_ip, command):
    try:
        ros = create_ros(robot_ip)
        print('command to execite', command)
        launch_command_topic = Topic(ros, '/launch_command',
              'std_msgs/String')
        # launch_command_topic.advertise()
        launch_command_topic.publish({'data': command  })
        return jsonify({"message": "Command sent successfully"})

    except Exception as e:
        return jsonify({"error": str(e)}), 500


def create_ros(robot_ip):
    if robot_ip in ros_connections:
        ros = ros_connections[robot_ip]
    else:
        ros = Ros(robot_ip, 9090)
        ros.run()
        ros_connections[robot_ip] = ros
    return ros

def create_topic(ros, topic_name):
    topic = Topic(ros, topic_name, 'std_msgs/String')
    topic.advertise()
    return topic

    