from unittest.mock import patch
from controllers import routes
import unittest
from extensions import socketio
from flask import Flask, request
from services import socket_manager
from classes import mission_room

class SocketManagerTest(unittest.TestCase):
    def setUp(self):
        self.app = Flask(__name__)
        self.app.register_blueprint(routes.main)
        self.app.config['TESTING'] = True
        self.socketio = socketio.init_app(self.app)
        self.socket_manager = socket_manager

        self.socket_manager.mission_rooms = {}
        self.robot_data = {
            "name": "Robot",
            "ipAddress": "0.0.0.0",
            "batteryLevel": 100
        }

    def tearDown(self):
        self.socket_manager.mission_rooms = {}

    @patch('services.socket_manager.join_room')
    @patch('services.socket_manager.emit')
    def test_create_mission_room(self, mock_emit, mock_join_room):
        expected_robot = mission_room.Robot("Robot", "0.0.0.0", 100)

        with self.app.test_request_context(environ_base={'REMOTE_ADDR': '127.0.0.1'}):
            request.environ['socketio'] = socketio.server
            request.sid = 'sid'
            self.socket_manager.create_mission_room(self.robot_data)
            expected_room = mission_room.MissionRoom(request.sid, self.robot_data)
        
        mock_join_room.assert_called_once_with(self.robot_data["ipAddress"])
        mock_emit.assert_called_once_with("createdMissionRoom", expected_room.to_dict(), broadcast=True)
        assert isinstance(self.socket_manager.mission_rooms[self.robot_data["ipAddress"]].host_id, str), "Argument is not a string"
        assert self.socket_manager.mission_rooms[self.robot_data["ipAddress"]].guest_id == [], "Incorrect guest_id value"
        assert self.socket_manager.mission_rooms[self.robot_data["ipAddress"]].robot_info.to_dict() == expected_robot.to_dict(), "Robot has unmatched values"


    @patch('services.socket_manager.emit')
    @patch('services.socket_manager.close_room')
    def test_handle_stop_mission(self, mock_close_room, mock_emit):
        room = mission_room.MissionRoom("host", self.robot_data)
        self.socket_manager.mission_rooms = {"0.0.0.0": room}
        with self.app.test_request_context():
            self.socket_manager.handle_stop_mission(self.robot_data)

        mock_close_room.assert_called_once_with('0.0.0.0')
        assert mock_emit.call_count == 2
        self.assertNotIn(self.robot_data["ipAddress"], self.socket_manager.mission_rooms)

    @patch('services.socket_manager.emit')
    def test_get_available_rooms(self, mock_emit):
        expected_room = mission_room.MissionRoom("host1", self.robot_data)
        self.socket_manager.mission_rooms = {"0.0.0.0": expected_room}
        self.socket_manager.get_available_rooms("id")
        mock_emit.assert_called_once_with("availableRooms", [expected_room.to_dict()], room=str("id"))


    @patch('services.socket_manager.join_room')
    @patch('services.socket_manager.emit')
    def test_view_mission_room(self, mock_emit, mock_join_room):
        robot = mission_room.MissionRoom("host", self.robot_data)
        self.socket_manager.mission_rooms = {'0.0.0.0': robot}

        with self.app.test_request_context(environ_base={'REMOTE_ADDR': '127.0.0.1'}):
            request.environ['socketio'] = socketio.server
            request.sid = 'random_sid'
            self.socket_manager.view_mission_room(self.robot_data)
        
        mock_join_room.assert_called_once_with('0.0.0.0')
        mock_emit.assert_called_once_with("addedAsViewer", self.socket_manager.mission_rooms['0.0.0.0'].to_dict(), room='0.0.0.0')
        assert self.socket_manager.mission_rooms['0.0.0.0'].guest_id == ["random_sid"], "Guest was not added to the mission room"


    @patch('services.socket_manager.handle_stop_mission')
    def test_handle_disconnect_host(self, mock_stop_mission):
        expected_room = mission_room.MissionRoom("host", self.robot_data)
        self.socket_manager.mission_rooms = {"0.0.0.0": expected_room}
        self.socket_manager.handle_disconnect("host")
        mock_stop_mission.assert_called_once_with(expected_room.robot_info.to_dict())

    @patch('services.socket_manager.leave_room')
    def test_handle_disconnect_guest(self, mock_leave_room):
        expected_room = mission_room.MissionRoom("host", self.robot_data)
        expected_room.add_guest("guest")
        self.socket_manager.mission_rooms = {"0.0.0.0": expected_room}
        self.socket_manager.handle_disconnect("guest")
        mock_leave_room.assert_called_once_with("0.0.0.0", "guest")
        assert self.socket_manager.mission_rooms["0.0.0.0"].guest_id == [], "Guest was not disconnected"


        

        

    

if __name__ == '__main__':
    unittest.main()
    # to run these tests, execute python -m unittest tests.test_socket_service
    