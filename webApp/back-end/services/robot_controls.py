import time
import roslibpy
from roslibpy import Topic, Ros
from flask import jsonify
from services import ros_utilities as ros


client = roslibpy.Ros(host='192.168.64.12', port=9090)

def identify_robot(robot_ip):
    if robot_ip == '198.162.0.122':
        return draw_on_screen(robot_ip)
    if robot_ip == '192.168.0.110':
        return play_sound(robot_ip)            

def connect_same_topic(i,speed,cmd_topic):
    cmd_topic.advertise()
    linear_speed = speed
    msg = roslibpy.Message({
            'linear': {'x': linear_speed, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 1.0, 'y': 0.0, 'z': 0.0}
        })
    cmd_topic.publish(roslibpy.Message(msg))
    time.sleep(1)

def simulate_mission(speed):
    try:
        client.run()
        for i in range(0, 3):
            cmd_topic = roslibpy.Topic(client, '/robot' + str(i) + '/cmd_vel', 'geometry_msgs/Twist')
            connect_same_topic(i,speed,cmd_topic)
    except Exception as e:
        print(f"An error occurred: {str(e)}")

def terminate_mission(speed):
    try:
        for i in range(0, 3):
            cmd_topic = roslibpy.Topic(client, '/robot' + str(i) + '/cmd_vel', 'geometry_msgs/Twist')
            connect_same_topic(i,speed,cmd_topic)
            cmd_topic.unadvertise()
        client.terminate()
    except Exception as e:
        print(f"An error occurred: {str(e)}")

def draw_on_screen(robot_ip):
    return ros.send_command(robot_ip, 'draw_on_screen.launch')

def play_sound(robot_ip):
    return ros.send_command(robot_ip, 'play_sound.launch')
  
