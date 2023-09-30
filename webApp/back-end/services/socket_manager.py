from flask_socketio import emit, join_room, leave_room, close_room
from flask import request
from classes.mission_room import MissionRoom

mission_rooms = {}

def create_mission_room(robot):
    try:
        join_room(robot["ipAddress"])
        mission_room = MissionRoom(request.sid, robot)
        mission_rooms[robot["ipAddress"]] = mission_room
        emit("createdMissionRoom", mission_room.to_dict(), broadcast=True)
    except Exception as e:
        print(f"An error occurred: {str(e)}")

def handle_stop_mission(robot):
    try:
        if robot["ipAddress"] in mission_rooms:
            emit("hostLeftRoom", room=robot["ipAddress"])
            emit("roomDeleted", broadcast=True)            
            del mission_rooms[robot["ipAddress"]]
            close_room(str(robot["ipAddress"]))    
    except KeyError:
        print("An error occurred: robotIp not found in missionRooms")

def get_available_rooms(id):
    try:
        rooms = [ room.to_dict() for room in mission_rooms.values()]
        emit("availableRooms", rooms, room=str(id))
    except Exception as e:
        print(f"An error occurred: {str(e)}")

def view_mission_room(robot):
    try:
        join_room(robot["ipAddress"])
        mission_rooms[robot["ipAddress"]].add_guest(request.sid)
        emit("addedAsViewer", mission_rooms[robot["ipAddress"]].to_dict(), room=str(robot["ipAddress"]))
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
