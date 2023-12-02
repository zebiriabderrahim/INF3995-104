from services import ros_utilities as ros
import roslibpy
from flask import jsonify
import json
import time

are_two_physical_launched = False

def identify_robot(robot):
    """
    Identifies a specific robot based on its IP address and triggers a sound command.

    Args:
    - robot (dict): A dictionary containing robot information, including the 'ipAddress' field.

    Returns:
    - The result of executing a sound command corresponding to the specified robot's IP address.
    - If the robot's IP address matches '192.168.0.122', it executes a specific sound command.
    - If the robot's IP address matches '192.168.0.110', it executes another sound command.
    """
    if robot["ipAddress"] == '192.168.0.122':
        return ros.execute_command(robot, 'play -n -c1 synth pluck C4 pluck E4 pluck G4 fade q 0.1 1 0.1')
    if robot["ipAddress"] == '192.168.0.110':
        return ros.execute_command(robot, 'play -n -c1 synth fade q 0.1 1 0.1')


def launch_mission(robot):
    """
    Launches a mission sequence on a specified robot.

    Args:
    - robot (dict): A dictionary containing robot information.

    """
    ros.execute_command(robot, 'roslaunch limo_bringup start_mission.launch')
    time.sleep(3)
    ros.execute_command(robot, 'rosrun recovery_algo recovery_algo.py')
    ros.execute_command(robot, 'rosrun environment_setting distance_calculator.py')


def launch_robots(robots):
    """
    Launches a sequence of missions on multiple robots.

    Args:
    - robots (list): A list containing dictionaries with information about multiple robots.

    Note:
    - This function orchestrates the launch sequence for missions on multiple robots.
    - It launches the 'map_merger' component required for mission execution via 'launch_map_merger()'.
    - Iterates through the list of robots provided and initiates the mission launch for each using 'launch_mission()'.

    """
    global are_two_physical_launched
    start_robots_communication()
    launch_map_merger()
    for robot in robots:
        launch_mission(robot)
    are_two_physical_launched = True  
     

def launch_map_merger():
    """
    Launches the 'merge.launch' file responsible for map merging on robot 1.

    """
    ros.execute_command({'ipAddress':'192.168.0.110'}, 'roslaunch merge_map merge.launch')


def start_robots_communication():
    """
    Initiates communication setup between robots for to merge the maps recieved between 2 robots.

    Note:
    - 'rosrun map_republisher map_republisher.py robot1' and 'robot2' republish maps for respective robots.
    - The IP addresses '192.168.0.110' and '192.168.0.122' correspond to the involved robots.
    - These commands are crucial for enabling communication and map republishing among robots.
    """
    ros.execute_command({'ipAddress':'192.168.0.110'}, 'roslaunch robot_communication robot_communication.launch')
    ros.execute_command({'ipAddress':'192.168.0.110'}, 'rosrun map_republisher map_republisher.py robot1')
    ros.execute_command({'ipAddress':'192.168.0.122'}, 'rosrun map_republisher map_republisher.py robot2')


def terminate_mission(robot):
    """
    Terminates the mission running on a specified robot.

    Args:
    - robot (dict): A dictionary containing information about the robot.

    """
    ros.terminate_mission(robot)
    