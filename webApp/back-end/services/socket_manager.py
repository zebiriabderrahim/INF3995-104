from flask_socketio import emit, join_room, leave_room, close_room
from flask import request
from classes.mission_room import MissionRoom
from services import ros_utilities,socket_service
import roslibpy
import time
import math
import numpy as np
from sklearn.cluster import DBSCAN


from services import socket_service

mission_rooms = {}
simulated_rooms = {}

map_resolution = 0.05  # Replace with your map's resolution
occupied_thresh = 0.65  # Replace with your map's occupied threshold
cluster_distance = 0.25  # Set your desired clustering distance

# Define a global variable to store the dictionary of obstacles
global_obstacle_dict = {}
origin = [-50, -50]
robot1_position = [0.0, 0.0]
robot2_position = [0.0, 0.0]

def odom_callback_robot1(msg):
    global robot_position
    # Mise à jour de la position du robot en utilisant les données d'odométrie
    robot1_position[0] = msg["pose"]["pose"]["position"]["x"]
    robot1_position[1] = msg["pose"]["pose"]["position"]["y"]

def odom_callback_robot2(msg):
    global robot_position
    # Mise à jour de la position du robot en utilisant les données d'odométrie
    robot2_position[0] = msg["pose"]["pose"]["position"]["x"]
    robot2_position[1] = msg["pose"]["pose"]["position"]["y"]

def map_callback(data, robot_ip):
    global global_obstacle_dict
    global robot_position


    # Extract map attributes using dictionary-style access
    width = data["info"]["width"]
    height = data["info"]["height"]
    data = data["data"]

    # Convert map data to a numpy array for processing
    occupancy_grid = np.array(data).reshape(height, width)

    socket_service.socketio.emit("map", data, room=robot_ip)

    # Identify obstacle cells
    obstacle_cells = np.argwhere(occupancy_grid > occupied_thresh)

    # Cluster obstacle cells using DBSCAN
    clustering = DBSCAN(eps=cluster_distance, min_samples=1).fit(obstacle_cells)
    cluster_labels = clustering.labels_

    for cluster_id in set(cluster_labels):
        if cluster_id == -1:
            continue  # Ignore noise points

        # Get cells in the cluster
        cluster_cells = obstacle_cells[cluster_labels == cluster_id]

        # Calculate the position of the obstacle
        centroid = np.mean(cluster_cells, axis=0)
        position = [(centroid[1] * map_resolution) + origin[0], (centroid[0] * map_resolution) + origin[1]]  # Convertir en coordonnées du monde réel

        # Calculate the distance to the obstacle (you'll need the robot's position for this)
        distance1 = np.linalg.norm(np.array(position) - np.array(robot1_position))
        distance2 = np.linalg.norm(np.array(position) - np.array(robot2_position))
        if(distance1 < distance2):
            distance = distance1
            robot_used = "robot1"
        else:
            distance = distance2
            robot_used = "robot2"

        if distance <= 4:
            # Check if this obstacle is close to any existing obstacle in the global_obstacle_dict
            merged = False
            position_tuple = tuple(position)  # Convert the list to a tuple
            for existing_position, existing_distance in global_obstacle_dict.items():
                if np.linalg.norm(np.array(existing_position) - np.array(position_tuple)) < cluster_distance:
                    # Merge the new obstacle with the existing one
                    existing_distance = min(existing_distance, distance)
                    merged = True
                    break

            if not merged:
                # Add the new obstacle to the dictionary
                global_obstacle_dict[position_tuple] = distance
                # round distance and position to 2 decimal places
                distance = round(distance, 2)
                position = [round(position[0], 2), round(position[1], 2)]
                print(f"Obstacle detected at {position} at {distance} m. de {robot_used}")
                socket_service.socketio.emit("log", {"type": "other", "name": "lidar", "message": f"Obstacle en {position} à {distance} m de {robot_used}", "timestamp": time.strftime("%b %d %H:%M:%S")}, room=robot_ip)

def create_mission_room(robot=None, type=None):
    try:
        if robot is None:
            room_name = 'simulation'
            mission_room = MissionRoom(request.sid)
            simulated_rooms[room_name] = mission_room
        else:
            if type == 'robot simulation':
                mission_room = MissionRoom(request.sid, robot, None, 'simulation')
                room_name = str(robot["ipAddress"] + 'sim')
                simulated_rooms[robot["ipAddress"]] = mission_room
            else:
                mission_room = MissionRoom(request.sid, robot)
                room_name = robot["ipAddress"]
                mission_rooms[robot["ipAddress"]] = mission_room
        
        join_room(room_name)
        socket_service.socketio.emit("createdMissionRoom", mission_room.to_dict())
        socket_service.socketio.emit("log", {"type": "system", "name": "system", "message": "Mission démarrée", "timestamp": time.strftime("%b %d %H:%M:%S")}, room=robot["ipAddress"])
    except Exception as e:
        print(f"An error occurred: {str(e)}")


def handle_stop_mission(robot):
    try:
        if robot["ipAddress"] in mission_rooms:
            socket_service.socketio.emit("log", {"type": "system", "name": "system", "message": "Mission arrêtée", "timestamp": time.strftime("%b %d %H:%M:%S")}, room=robot["ipAddress"])
            socket_service.socketio.emit("hostLeftRoom", room=robot["ipAddress"])
            socket_service.socketio.emit("roomDeleted")            
            del mission_rooms[robot["ipAddress"]]
            close_room(str(robot["ipAddress"]))    
    except KeyError:
        print("An error occurred: robotIp not found in missionRooms")

def stop_simulation(robot, type=None):
    try:
        if type == 'simulation':
            room_name = 'simulation'
        else:
            room_name = str(robot["ipAddress"] + 'sim')
        
        socket_service.socketio.emit("hostLeftRoom", room_name)
        socket_service.socketio.emit("roomDeleted") 
        socket_service.socketio.emit('stoppedSimulation', room=room_name)
        if type == 'simulation':
            del simulated_rooms[room_name]
        else:
            del simulated_rooms[robot["ipAddress"]]
        close_room(room_name)
    
    except Exception as e:
        print(f"An error occurred: {str(e)}")


def get_available_rooms():
    try:
        sim_rooms = [room.to_dict() for room in simulated_rooms.values()]
        rooms = [room.to_dict() for room in mission_rooms.values()]
        socket_service.socketio.emit("availableRooms", {'rooms': rooms, 'simulated': sim_rooms})
    except Exception as e:
        print(f"An error occurred: {str(e)}")

def view_mission_room(robot):
    try:
        join_room(robot["ipAddress"])
        mission_rooms[robot["ipAddress"]].add_guest(request.sid)
        socket_service.socketio.emit("addedAsViewer", mission_rooms[robot["ipAddress"]].to_dict(), room=str(robot["ipAddress"]))
    except Exception as e:
        print(f"An error occurred: {str(e)}")

def send_log(robot_ip):
    try:
        if robot_ip in ros_utilities.ros_connections:
            ros = ros_utilities.ros_connections[robot_ip]
        else:
            ros = roslibpy.Ros(robot_ip, 9090)
            ros.run()
            ros_utilities.ros_connections[robot_ip] = ros
        pos_topic = roslibpy.Topic(ros, '/map', 'nav_msgs/OccupancyGrid')
        pos_topic_odom_robot1 = roslibpy.Topic(ros, '/robot1/odom', 'nav_msgs/Odometry')
        pos_topic_odom_robot2 = roslibpy.Topic(ros, '/robot2/odom', 'nav_msgs/Odometry')
        pos_topic.subscribe(lambda message: map_callback(message, robot_ip))
        pos_topic_odom_robot1.subscribe(odom_callback_robot1)
        pos_topic_odom_robot2.subscribe(odom_callback_robot2)
    except Exception as e:
        print(f"An error occurred: log couldn't be sent, {str(e)}")

def send_robot_battery(robot_ip, data):
    try:
        batteryLevel = round(data['data'])
        if (batteryLevel < 20):
            socket_service.socketio.emit("log", {"type": "system", "name": "system", "message": f"Niveau de batterie faible: {batteryLevel}%", "timestamp": time.strftime("%b %d %H:%M:%S")}, room=robot_ip)
        robot = {
            "ipAddress": robot_ip,
            "batteryLevel": batteryLevel
        }
        socket_service.socketio.emit("robotBattery", robot)
    except Exception as e:
        print(f"An error occurred: {str(e)}")

def handle_disconnect(id):
    try:
        for room in mission_rooms.values():
            if room.host_id == id:
                handle_stop_mission(room.robot_info.to_dict())       
                break
            elif id in room.guest_id:
                mission_rooms[room.robot_info.ip_address].guest_id.remove(id)
                leave_room(room.robot_info.ip_address, id)
                break
    except Exception as e:
        print(f"An error occurred: {str(e)}")






