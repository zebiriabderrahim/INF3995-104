from flask import request
from extensions import socketio
from services import socket_manager, robot_simulation, ros_utilities

@socketio.on("connect")
def handle_connect():

    print("Client connected!")

@socketio.on("createMissionRoom")
def handle_create_mission(robot):
    socket_manager.create_mission_room(robot)

@socketio.on("getAvailableRooms")
def get_available_rooms():
    socket_manager.get_available_rooms()

@socketio.on("viewMissionRoom")
def handle_view_mission_room(robot):
    socket_manager.view_mission_room(robot)
    
@socketio.on("stopMission")
def handle_stop_mission(robot):
    print("socket stop ")
    socket_manager.handle_stop_mission(robot)
    ros_utilities.terminate_mission(robot)

@socketio.on("getBatteryLevel")
def handle_get_battery_level(robot_ip):
    socketio.emit("stopBatteryCall", True)
    ros_utilities.subscribe_to_battery(robot_ip)

@socketio.on("disconnect")
def handle_disconnect():
    socket_manager.handle_disconnect(request.sid)

@socketio.on('simulateMission')
def handle_simulate_mission():
    socket_manager.create_mission_room()
    robot_simulation.simulate_robot_mission()

@socketio.on('simulateMissionRobot')
def handle_simulate_robot_mission(robot):
    socket_manager.create_mission_room(robot, 'robot simulation')
    robot_simulation.simulate_robot_mission(robot)

@socketio.on('stopSimulationRobot')
def handle_terminate_simulation_Robot(robot):
    robot_simulation.terminate_mission_robot(robot)
    if robot["ipAddress"] == 'simulation': 
        socket_manager.stop_simulation(robot, 'simulation')
        socket_manager.get_available_rooms()
    else: 
        socket_manager.stop_simulation(robot)

@socketio.on("getLogs")
def get_logs(robot_ip):
    socket_manager.send_log(robot_ip)
