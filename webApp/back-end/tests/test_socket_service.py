from unittest.mock import patch
from services import socket_manager, socket_service
from controllers import routes
from flask import Flask, jsonify
import unittest
from io import StringIO
from extensions import socketio
from flask import request

class SocketServiceTest(unittest.TestCase):
    def setUp(self):
        self.app = Flask(__name__)
        self.app.register_blueprint(routes.main)
        self.app.config['TESTING'] = True
        socketio.init_app(self.app, cors_allowed_origins="*")
        self.socketio_client = socketio.test_client(self.app)

        self.robot_data = {
            "name": "Robot",
            "ipAddress": "0.0.0.0",
            "battery": 100
        }
        
    def tearDown(self):
        self.socketio_client.disconnect()

    @patch('sys.stdout', new_callable=StringIO)
    def test_handle_connect(self, mock_stdout):
        self.socketio_client.emit("connect")
        self.assertEqual(mock_stdout.getvalue().strip(), "Client connected!")

    @patch('services.socket_manager.create_mission_room')
    def test_handle_create_mission(self, mock_create_mission_room):
        self.socketio_client.emit("createMissionRoom", self.robot_data)
        mock_create_mission_room.assert_called_with(self.robot_data)

    @patch('services.socket_manager.get_available_rooms')
    def test_handle_get_available_rooms(self, mock_get_available_rooms):
        self.socketio_client.emit('getAvailableRooms')
        mock_get_available_rooms.assert_called_once()
        args = mock_get_available_rooms.call_args[0]
        assert isinstance(args[0], str), "Argument is not a string"

    @patch('services.socket_manager.view_mission_room')
    def test_handle_view_mission_room(self, mock_view_mission_room):
        self.socketio_client.emit("viewMissionRoom", self.robot_data)
        mock_view_mission_room.assert_called_with(self.robot_data)

    @patch('services.socket_manager.handle_stop_mission')
    def test_handle_stop_mission(self, mock_stop_mission):
        self.socketio_client.emit("stopMission", self.robot_data)
        mock_stop_mission.assert_called_with(self.robot_data)

    @patch('services.socket_manager.handle_disconnect')
    def test_handle_get_available_rooms(self, mock_handle_disconnect):
        self.socketio_client.emit("disconnect")
        mock_handle_disconnect.assert_called_once()
        args = mock_handle_disconnect.call_args[0]
        assert isinstance(args[0], str), "Argument is not a string"


if __name__ == '__main__':
    unittest.main()
    # to run these tests, execute python -m unittest tests.test_socket_service
