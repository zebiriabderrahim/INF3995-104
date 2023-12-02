import dis
import time
from roslibpy import Topic, Ros
from flask import jsonify
from services import socket_manager, robot_simulation
from functools import partial

ros_connections = {}
robots_ip = []

def terminate_mission(robot):
    """
    Terminates the ongoing mission for a specified robot.

    This function interacts with the ROS system running on the specified robot's IP address.
    It unsubscribes from specific topics related to odometry and map data to halt the ongoing mission.
    Additionally, it executes a command to stop the mission execution.

    :param robot: A dictionary containing information about the robot including its IP address.
    """
    ros = create_ros(robot["ipAddress"])
    Topic(ros, "/odom", "nav_msgs/Odometry").unsubscribe()
    Topic(ros, "/map", "nav_msgs/OccupancyGrid").unsubscribe()

    # def distance_callback(message):
    #     robot_simulation.distance_callback(robot["name"], message,True)
    #     distance_topic.unsubscribe()  # Unsubscribe after receiving the first message
    
    # distance_topic = Topic(ros, "/distance", "std_msgs/Float64")
    # distance_topic.subscribe(distance_callback)
    
    execute_command(robot, f"rosrun stop_mission stop_commander.py")


def return_robot_to_base(robot_ip):
    """
    Initiates the return of a robot to its designated base.

    This function executes a ROS command on the specified robot with the given IP address.
    The command triggers a script, 'return_base.py', responsible for returning the robot to its base.

    :param robot_ip: The IP address of the robot to return to base.
    :return: None or a JSON response with an error message and HTTP status code if an exception occurs.
    """
    try:
        execute_command({"ipAddress": robot_ip}, f"rosrun return_base return_base.py")
    
    except Exception as e:
        return jsonify({"error on subscribe to battery": str(e)}), 500


def subscribe_to_battery(robot):
    """
    Subscribes to the battery status of a given robot.

    This function uses ROS commands to initiate the battery status monitoring on the specified robot.
    It launches a 'battery.launch' ROS launch file to start the battery monitoring process.
    The function then subscribes to the '/battery_percentage' topic to receive real-time battery percentage updates.
    Battery updates are sent to a socket room using the 'socket_manager.send_robot_battery' callback.

    :param robot: A dictionary containing information about the robot, including its IP address.
    :return: A JSON response with a success message or an error message and HTTP status code if an exception occurs.
    """
    try:
        ros = create_ros(robot["ipAddress"])
        execute_command(robot, f"roslaunch battery battery.launch")
        Topic(ros, "/battery_percentage", "std_msgs/Float32").subscribe(partial(socket_manager.send_robot_battery, robot["ipAddress"]))

        return jsonify({"message": "Command sent successfully"})

    except Exception as e:
        return jsonify({"error on subscribe to battery": str(e)}), 500


def execute_command(robot, command):
    """
    Executes a command on a specified robot using ROS.

    This function creates a ROS connection to the specified robot's IP address.
    It publishes a command to the '/launch_command' topic, allowing remote execution of ROS commands.
    The function sends the specified command to the robot using ROS communication.

    :param robot: A dictionary containing information about the robot, including its IP address.
    :param command: The command to be executed on the robot via ROS.
    :return: A JSON response with a success message or an error message and HTTP status code if an exception occurs.
    """
    try:
        ros = create_ros(robot["ipAddress"])
        launch_command_topic = Topic(ros, f"/launch_command", "std_msgs/String")
        launch_command_topic.publish({"data": command})
        return jsonify({"message": "Command sent successfully"})

    except Exception as e:
        return jsonify({"error on execute command": str(e)}), 500


def create_ros(robot_ip):
    """
    Creates or retrieves a ROS connection for a specified robot IP address.

    This function checks if a ROS connection for the given robot IP address already exists.
    If an existing connection is found, it is returned. Otherwise, a new ROS connection is created.
    The new connection is started by invoking the 'run()' method.

    :param robot_ip: The IP address of the robot for which a ROS connection is needed.
    :return: A ROS connection object for the specified robot IP address.
    """
    if robot_ip in ros_connections:
        ros = ros_connections[robot_ip]
    else:
        ros = Ros(robot_ip, 9090)
    ros.run()
    ros_connections[robot_ip] = ros
    return ros


def create_topic(ros, topic_name):
    """
    Creates and advertises a specified topic within the provided ROS environment.

    :param ros: ROS instance representing the Robot Operating System connection.
    :param topic_name: Name of the topic to be created within the ROS environment.
    :return: An instance of the created topic object.
    
    """
    topic = Topic(ros, topic_name, "std_msgs/String")
    topic.advertise()
    return topic
