from tracemalloc import stop
from flask_socketio import emit, join_room, leave_room, close_room
from flask import request
from classes.mission_room import MissionRoom
from services import ros_utilities,socket_service,robot_simulation
import roslibpy
import time
import numpy as np
from sklearn.cluster import DBSCAN
from services import socket_service
import numpy as np
from scipy.ndimage import label

mission_rooms = {}
simulated_rooms = {}
map_resolution = 0.05  
occupied_thresh = 0.65  
cluster_distance = 0.25  
global_obstacle_dict = {}
global_obstacle_dict2={}
origin = [-5, -5]
last_processed_time = 0
robot1_position = [0.0, 0.0]
robot2_position = [0.0, 0.0]
is_battery_low = {"192.168.0.110": False, "192.168.0.122": False}

def odom_callback_robot1(msg, robot, room_name):
    """
    Updates Robot1's position and emits its odometry data to a specific socket room.

    :param msg: Odometry message with pose and position data for Robot1.
    :param robot: Dictionary with Robot1's information (e.g., name, details).
    :param room_name: Name of the socket room to emit the position data.
    """
    global robot1_position

    robot1_position[0] = msg["pose"]["pose"]["position"]["x"]
    robot1_position[1] = msg["pose"]["pose"]["position"]["y"]

    socket_service.socketio.emit("recieveSimRobotPos",{"robotId": f'{robot["name"].lower().replace(" ", "")[-1]}', "position": msg["pose"]["pose"]["position"]}, room=room_name) 


def odom_callback_robot2(msg, robot, room_name):
    """
    Updates Robot2's position and emits its odometry data to a specific socket room.

    :param msg: Odometry message with pose and position data for Robot2.
    :param robot: Dictionary with Robot2's information (e.g., name, details).
    :param room_name: Name of the socket room to emit the position data.
    """
    global robot2_position

    robot2_position[0] = msg["pose"]["pose"]["position"]["x"]
    robot2_position[1] = msg["pose"]["pose"]["position"]["y"]
    socket_service.socketio.emit("recieveSimRobot2Pos",{"robotId": f'{robot["name"].lower().replace(" ", "")[-1]}', "position": msg["pose"]["pose"]["position"]}, room=room_name)


def map_callback(data, robot_ip):
    """
    Processes map data and emits information about detected obstacles to the specified socket room.

    :param data: Map data containing information about the environment.
    :param robot_ip: IP address or identifier of the robot for which the map data is processed.
    """
    global global_obstacle_dict

    width = data["info"]["width"]
    height = data["info"]["height"]
    data = data["data"]

    occupancy_grid = np.array(data).reshape(height, width)
    socket_service.socketio.emit("map", data, room=robot_ip)

    obstacle_cells = np.argwhere(occupancy_grid > occupied_thresh)
    clustering = DBSCAN(eps=cluster_distance, min_samples=1).fit(obstacle_cells)

    cluster_labels = clustering.labels_
    for cluster_id in set(cluster_labels):
        if cluster_id == -1:
            continue  

        cluster_cells = obstacle_cells[cluster_labels == cluster_id]

        centroid = np.mean(cluster_cells, axis=0)
        position = [(centroid[1] * map_resolution) + origin[0], (centroid[0] * map_resolution) + origin[1]]  

        robot_used = ""

        if(robot_ip == 'physical'):
            distance1 = np.linalg.norm(np.array(position) - np.array(robot1_position))
            distance2 = np.linalg.norm(np.array(position) - np.array(robot2_position))
            if(distance1 < distance2):
                distance, robot_used = distance1, "robot 1"
            else:
                distance, robot_used = distance2, "robot 2"

        elif(robot_ip == "simulation"):
            distance1 = np.linalg.norm(np.array(position) - np.array(robot_simulation.current_positions[0]))
            distance2 = np.linalg.norm(np.array(position) - np.array(robot_simulation.current_positions[1]))
            if(distance1 < distance2):
                distance = distance1
                robot_used = "robot 1"
            else:
                distance = distance2
                robot_used = "robot 2"

        elif(robot_ip == "192.168.0.110"):
            distance = np.linalg.norm(np.array(position) - np.array(robot1_position))

        elif(robot_ip == "192.168.0.122"):
            distance = np.linalg.norm(np.array(position) - np.array(robot2_position))

        elif(robot_ip == "192.168.0.110sim"):
            distance = np.linalg.norm(np.array(position) - np.array(robot_simulation.current_positions[0]))

        elif(robot_ip == "192.168.0.122sim"):
            distance = np.linalg.norm(np.array(position) - np.array(robot_simulation.current_positions[1]))

        if distance <= 4:
            merged = False
            position_tuple = tuple(position)  
            for existing_position, existing_distance in global_obstacle_dict.items():
                if np.linalg.norm(np.array(existing_position) - np.array(position_tuple)) < cluster_distance:
                    existing_distance = min(existing_distance, distance)
                    merged = True
                    break

            if not merged:
                global_obstacle_dict[position_tuple] = distance
                distance = round(distance, 2)
                position = [round(position[0], 2), round(position[1], 2)]
                socket_service.socketio.emit("log", {"type": "other", "name": "lidar", "message": f"Obstacle en {position} à {distance} m {robot_used}", "timestamp": time.strftime("%b %d %H:%M:%S")}, room=robot_ip)


def create_mission_room(robot=None, type=None):
    """
    Creates a mission room for robots in a simulated of physical context and emits room creation messages.
    
    :param robot: Dictionary containing information about the robot, including IP address and details.
    :param type: Type of mission room ('robot simulation', 'physical', or None).
    """
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
            elif type == 'physical':
                mission_room = MissionRoom(request.sid, robot, None, 'physical')
                room_name = 'physical'
                mission_rooms[room_name] = mission_room
            else:
                mission_room = MissionRoom(request.sid, robot)
                room_name = robot["ipAddress"]
                mission_rooms[robot["ipAddress"]] = mission_room
        
        join_room(room_name)
        socket_service.socketio.emit("createdMissionRoom", mission_room.to_dict(),room=room_name)
        socket_service.socketio.emit("roomCreated", mission_room.to_dict())
        socket_service.socketio.emit("log", {"type": "system", "name": "system", "message": "Mission démarrée", "timestamp": time.strftime("%b %d %H:%M:%S")}, room=room_name)
    except Exception as e:
        print(f"An error occurred in create_mission_room function: {str(e)}")


def handle_stop_mission(robot=None):
    """
    Handles stopping a mission for robots during physical exploration.

    :param robot: Dictionary containing information about the robot, including IP address and details.
    """
    try:
        if robot is None:
            room_name = 'physical'
            message = "Les deux robots physiques"
        elif robot["ipAddress"] in mission_rooms:
            room_name = robot["ipAddress"]
            message = f"Le robot {robot['name'].lower().replace(' ', '')[-1]} en exploration physique"
            
        socket_service.socketio.emit("log", {"type": "system", "name": "system", "message": "Mission arrêtée", "timestamp": time.strftime("%b %d %H:%M:%S")}, room=room_name)
        socket_service.socketio.emit("hostLeftRoom", room=room_name)
        socket_service.socketio.emit("roomDeleted", message)
                            
        del mission_rooms[room_name]
    except Exception as e:
        print(f"An error occurred in handle stop mission function:{str(e)}")


def stop_simulation(robot, type=None):
    """
    Stops a simulation for a robot or both robots in simulation mode.

    :param robot: Dictionary containing information about the robot, including IP address and details.
    :param type: Type of simulation ('simulation' or None).
    """  
    try:
        if type == 'simulation':
            room_name = 'simulation'
            message = "Les deux robots en simulation"
        else:
            room_name = str(robot["ipAddress"]) + 'sim'
            message = f"Le robot {robot['name'].lower().replace(' ', '')[-1]} en simulation"
        
        socket_service.socketio.emit("log", {"type": "system", "name": "system", "message": "Simulation arrêtée", "timestamp": time.strftime("%b %d %H:%M:%S")}, room=room_name)
        socket_service.socketio.emit("hostLeftRoom", room=room_name)
        socket_service.socketio.emit("roomDeleted", message,room=room_name) 
        socket_service.socketio.emit("receiveDistanceSim", robot_simulation.simulated_robot_distance, room=room_name)
        socket_service.socketio.emit('stoppedSimulation', room=room_name)

        if type == 'simulation':   
            del simulated_rooms[room_name]     
        else:
            del simulated_rooms[robot["ipAddress"]]
        close_room(room_name)
    
    except Exception as e:
        print(f"An error occurred in stop_simulation function: {str(e)}")


def get_available_rooms():
    """
    Retrieves available mission and simulation rooms.

    - Gathers information about available mission rooms and simulation rooms.
    - Emits a message containing details about available rooms to the appropriate socket.
    """
    try:
        sim_rooms = [room.to_dict() for room in simulated_rooms.values()]
        rooms = [room.to_dict() for room in mission_rooms.values()]
        socket_service.socketio.emit("availableRooms", {'rooms': rooms, 'simulated': sim_rooms})
    except Exception as e:
        print(f"An error occurred in get_available_rooms function: {str(e)}")


def view_mission_room(robot, is_simulation=False):
    """
    Allows viewing a mission either in simulation or occurring on the physical robots.

    :param robot: Dictionary containing information about the robot, including IP address and details.
    :param is_simulation: Flag indicating whether it's a simulation room (default: False).
    """
    try:
        if is_simulation:
            join_room(str(robot["ipAddress"]) + 'sim')
            simulated_rooms[robot["ipAddress"]].add_guest(request.sid)
            socket_service.socketio.emit("addedAsViewer", simulated_rooms[robot["ipAddress"]].to_dict(), room=str(robot["ipAddress"]) + 'sim')
        else:
            join_room(robot["ipAddress"])
            mission_rooms[robot["ipAddress"]].add_guest(request.sid)
            socket_service.socketio.emit("addedAsViewer", mission_rooms[robot["ipAddress"]].to_dict(), room=str(robot["ipAddress"]))
    except Exception as e:
        print(f"An error occurred in view_mission_room function: {str(e)}")


def send_log(robots, all_robots = False):
    """
    Sends logs and subscribes to ROS topics for robot data processing.

    :param robots: List containing dictionaries with information about the robot(s).
    :param all_robots: Flag to indicate processing logs and topics for all robots (default: False).
    """
    try:
        if all_robots:
            ros = ros_utilities.create_ros('192.168.0.110')
            ros_robot2 = ros_utilities.create_ros('192.168.0.122')
            roslibpy.Topic(ros, '/robot1/odom', 'nav_msgs/Odometry').subscribe(lambda message: odom_callback_robot1(message, robots[0], 'physical'))
            roslibpy.Topic(ros_robot2, '/robot2/odom', 'nav_msgs/Odometry').subscribe(lambda message: odom_callback_robot2(message, robots[1], 'physical'))
            roslibpy.Topic(ros, '/merged_map', 'nav_msgs/OccupancyGrid').subscribe(lambda message: map_callback(message, 'physical')) 
        else:
            ros = ros_utilities.create_ros(robots[0]['ipAddress'])
            roslibpy.Topic(ros, '/map', 'nav_msgs/OccupancyGrid').subscribe(lambda message: map_callback(message, robots[0]['ipAddress']))
            if robots[0]['ipAddress'] == '192.168.0.110': roslibpy.Topic(ros, '/odom', 'nav_msgs/Odometry').subscribe(lambda message: odom_callback_robot1(message, robots[0], '192.168.0.110'))
            else: roslibpy.Topic(ros, '/odom', 'nav_msgs/Odometry').subscribe(lambda message: odom_callback_robot2(message, robots[0], '192.168.0.122'))

    except Exception as e:
        print(f"An error occurred in send_log function: log couldn't be sent, {str(e)}")


def send_robot_battery(robot_ip, data):
    """
    Sends battery level data for a robot and emits low battery alerts if applicable.

    :param robot_ip: IP address of the robot.
    :param data: Dictionary containing battery level data.
    """
    try:
        battery_level = round(data['data'])
        robot = {
            "ipAddress": robot_ip,
            "batteryLevel": battery_level
        }
    
        socket_service.socketio.emit("robotBattery", robot)

        if battery_level < 30 and not is_battery_low[robot_ip]:
            is_battery_low[robot_ip] = True
            socket_service.socketio.emit("log", {"type": "system", "name": "system", "message": f"Niveau de batterie faible: {battery_level}%", "timestamp": time.strftime("%b %d %H:%M:%S")}, room=robot_ip)

    except Exception as e:
        print(f"An error occurred in send_robot_battery function: {str(e)}")


def handle_disconnect(id):
    """
    Handles disconnection events for users in mission or simulation rooms.
    
    :param id: Identifier for the disconnected user.
    """
    try:
        for room in mission_rooms.values():
            if room.host_id == id:
                handle_stop_mission(room.robot_info.to_dict())       
                break
            elif id in room.guest_id:
                mission_rooms[room.robot_info.ip_address].guest_id.remove(id)
                leave_room(room.robot_info.ip_address, id)
                break
            
        for room in simulated_rooms.values():
            if room.host_id == id:
                if room.robot_info.ip_address == 'simulation':
                    robot_simulation.terminate_mission_robot()
                    stop_simulation(room.robot_info.to_dict(), 'simulation') 
                else :
                    robot_simulation.terminate_mission_robot(room.robot_info.to_dict())
                    stop_simulation(room.robot_info.to_dict())    
                break   
    except Exception as e:
        print(f"An error occurred on handle disconnect: {str(e)}")


def view_all_robots():
    """
    Allows viewing the ongoing mission for all physical robots.

    """
    try:
        join_room('physical')
        mission_rooms['physical'].add_guest(request.sid)
        socket_service.socketio.emit("addedAsViewer", mission_rooms['physical'].to_dict(), room=str('physical'))
    except Exception as e:
        print(f"An error occurred on view mission room -- all physical robots: {str(e)}")
                







