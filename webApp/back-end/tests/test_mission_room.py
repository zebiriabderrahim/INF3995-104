import unittest
from flask import Flask
from controllers import routes
from classes import mission_room
from unittest.mock import patch

class MissionRoomTest(unittest.TestCase):
    def setUp(self):
        self.app = Flask(__name__)
        self.app.register_blueprint(routes.main)
        self.app.config['TESTING'] = True
        self.client = self.app.test_client()

        self.robot_data = {
            "name": "Robot",
            "ipAddress": "0.0.0.0",
            "batteryLevel": 100
        }

        self.robot_data_physical = [{
            "name": "Robot",
            "ipAddress": "0.0.0.0",
            "batteryLevel": 100
        }, {
            "name": "Robot",
            "ipAddress": "0.0.0.0",
            "batteryLevel": 100
        }]

        self.mission_room = mission_room.MissionRoom('host', self.robot_data)
        self.robot = mission_room.Robot("Robot2", '0.0.0.1', '', 70)
        self.physical_mission_room = mission_room.MissionRoom('host', self.robot_data_physical, None, 'physical')
        self.simulation_mission_room = mission_room.MissionRoom('host', self.robot_data, None, 'simulation')
        self.one_simulated_robot_mission = mission_room.MissionRoom('host', None)

    def test_add_guest(self):
        self.mission_room.add_guest('guest')
        assert self.mission_room.guest_id == ['guest'], "Guest was not added to the mission room"

    @patch('classes.mission_room.Robot.to_dict')
    def test_mission_to_dict(self, mock_robot_to_dict):
        mock_robot_to_dict.return_value = 'robot_info'
        mission_dict = self.mission_room.to_dict()
        self.assertEqual(mission_dict, {
            'hostId': 'host',
            'robot': 'robot_info',
            'guestId': []
        }, 'Mission room info is incorrect')

    
    @patch('classes.mission_room.Robot.to_dict')
    def test_mission_to_dict_v2(self, mock_robot_to_dict):
        mock_robot_to_dict.return_value = 'robot_info'
        mission_dict = self.physical_mission_room.to_dict()
        self.assertEqual(mission_dict, {
            'hostId': 'host',
            'robot': 'robot_info',
            'guestId': [],
            'otherRobots': ['robot_info']
        }, 'Mission room info is incorrect')

    @patch('classes.mission_room.Robot.to_dict')
    def test_mission_to_dict_v3(self, mock_robot_to_dict):
        mock_robot_to_dict.return_value = 'robot_info'
        self.mission_room.robot_info = [self.robot]
        mission_dict = self.mission_room.to_dict()
        self.assertEqual(mission_dict, {
            'hostId': 'host',
            'robot': 'robot_info',
            'guestId': [],
            'otherRobots': []
        }, 'Mission room info is incorrect')

    def test_change_robot_state(self):
        self.robot.change_robot_state('Off')
        assert self.robot.state == 'Off', 'State was not updated'

    def test_robot_to_dict(self):
        robot_dict = self.robot.to_dict()
        self.assertEqual(robot_dict, {
            'name': 'Robot2',
            'ipAddress': '0.0.0.1',
            'state': 'Active on mission',
            'batteryLevel': 70
        }, "Robot info is incorrect")
        
if __name__ == '__main__':
    unittest.main()