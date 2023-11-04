from services import ros_utilities as ros
import roslibpy
from flask import jsonify

def draw_on_screen(robot_ip):
    return ros.execute_command(robot_ip, 'pacmd set-default-sink alsa_output.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo && pacmd set-sink-volume 1 0x10000 && roslaunch /home/nvidia/agilex_ws/src/identification/launch/play_sound.launch')

def play_sound(robot_ip):
    return ros.execute_command(robot_ip, 'pacmd set-default-sink alsa_output.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo && pacmd set-sink-volume 1 0x10000 && roslaunch /home/nvidia/agilex_ws/src/identification/launch/play_sound.launch')

def identify_robot(robot_ip):
    if robot_ip == '192.168.0.122':
        return draw_on_screen(robot_ip)
    if robot_ip == '192.168.0.110':
        return play_sound(robot_ip)    
    return jsonify({"error" : "no robot ip fouund"}), 500 

def launch_mission(robot_ip):
    if robot_ip == '192.168.0.110':
        return ros.launch_mission(robot_ip, 'roslaunch /home/nvidia/agilex_ws/src/limo_ros/limo_bringup/launch/testor.launch')
    if robot_ip == '192.168.0.122':
        return ros.launch_mission(robot_ip, 'roslaunch /home/nvidia/agilex_ws/src/limo_ros/limo_bringup/launch/www.launch')
    return jsonify({"error" : "no robot ip fouund"}), 500 

def terminate_mission(robot):
    return ros.terminate_mission(robot)
