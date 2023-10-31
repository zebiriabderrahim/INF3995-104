from services import ros_utilities as ros
import roslibpy
from flask import jsonify



def draw_on_screen(robot_ip):
    return ros.send_command(robot_ip, 'identification/launch/draw_screen.launch')

def play_sound(robot_ip):
    return ros.send_command(robot_ip, 'identification/launch/play_sound.launch')

def identify_robot(robot_ip):
    if robot_ip == '192.168.0.122':
        return draw_on_screen(robot_ip)
    if robot_ip == '192.168.0.110':
        return play_sound(robot_ip)    
    return jsonify({"error" : "no robot ip fouund"}), 500 

def launch_mission(robot_ip):
    return ros.launch_mission(robot_ip, 'launch_mission/src/launch_mission.launch')