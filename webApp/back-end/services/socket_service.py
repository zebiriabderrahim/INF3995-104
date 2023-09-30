from flask import request
from extensions import socketio
from services import socket_manager

@socketio.on("connect")
def handle_connect():
    print("Client connected!")

@socketio.on("createMissionRoom")
def handleCreateMission(robot):
    socket_manager.create_mission_room(robot)

@socketio.on("getAvailableRooms")
def getAvailableRooms():
    socket_manager.get_available_rooms(request.sid)

@socketio.on("viewMissionRoom")
def handleViewMissionRoom(robot):
    socket_manager.view_mission_room(robot)
    
@socketio.on("stopMission")
def handleStopMission(robot):
    socket_manager.handle_stop_mission(robot)

@socketio.on("disconnect")
def handleDisconnect():
    socket_manager.handle_disconnect(request.sid)

    