import json
import unittest
from unittest.mock import patch, Mock, call
import roslibpy
from flask import Flask
from flask import jsonify
from controllers import routes
from services import robot_controls 

class RobotControls(unittest.TestCase):
    def setUp(self):
        self.app = Flask(__name__)
        self.app.register_blueprint(routes.main)
        self.app.config['TESTING'] = True
        self.client = self.app.test_client()
        self.robot_controls = robot_controls

    @patch('services.ros_utilities.execute_command')
    def test_identify_robot1(self, mock_execute_command):
        with self.app.app_context():
            mock_execute_command.return_value = 'Successfully called the execute command function'
            robot1_result = self.robot_controls.identify_robot({'ipAddress': '192.168.0.110'})  
            mock_execute_command.assert_called_once_with({'ipAddress': '192.168.0.110'}, 'play -n -c1 synth fade q 0.1 1 0.1') 
            assert robot1_result == 'Successfully called the execute command function', 'execute_command was not called via identify(ip robot 1)'

    @patch('services.ros_utilities.execute_command')
    def test_identify_robot2(self, mock_execute_command):
        mock_execute_command.return_value = 'Successfully called the execute command function'
        robot2_result = self.robot_controls.identify_robot({'ipAddress': '192.168.0.122'})
        mock_execute_command.assert_called_once_with({'ipAddress': '192.168.0.122'}, 'play -n -c1 synth pluck C4 pluck E4 pluck G4 fade q 0.1 1 0.1')
        assert robot2_result == 'Successfully called the execute command function', 'execute_command was not called via identify(ip robot 2)'

    @patch('services.ros_utilities.execute_command')
    def test_launch_mission(self, mock_execute_command):
        self.robot_controls.launch_mission({'ipAddress': '192.168.0.110'})
        mock_execute_command.assert_has_calls([
            call({'ipAddress': '192.168.0.110'}, 'roslaunch limo_bringup start_mission.launch'),
            call({'ipAddress': '192.168.0.110'}, 'rosrun recovery_algo recovery_algo.py'),
            call({'ipAddress': '192.168.0.110'}, 'rosrun environment_setting distance_calculator.py')
        ])

    @patch('services.robot_controls.launch_mission')
    @patch('services.robot_controls.launch_map_merger')
    @patch('services.robot_controls.start_robots_communication')
    def test_launch_robots(self, mock_start_robots, mock_map_merger, mock_launch_mission):
        self.robot_controls.launch_robots(['robot1', 'robot2'])
        mock_start_robots.assert_called_once()
        mock_map_merger.assert_called_once()
        self.assertTrue(self.robot_controls.are_two_physical_launched, True)
        mock_launch_mission.assert_has_calls([
            call('robot1'),
            call('robot2')
        ])

    @patch('services.ros_utilities.execute_command')
    def test_launch_map_merger(self, mock_execute_command):
        self.robot_controls.launch_map_merger()
        mock_execute_command.assert_called_once_with({'ipAddress':'192.168.0.110'}, 'roslaunch merge_map merge.launch')

    @patch('services.ros_utilities.execute_command')
    def test_start_robots_communication(self, mock_execute_command):
        self.robot_controls.start_robots_communication()
        mock_execute_command.assert_has_calls([
            call({'ipAddress':'192.168.0.110'}, 'roslaunch robot_communication robot_communication.launch'),
            call({'ipAddress':'192.168.0.110'}, 'rosrun map_republisher map_republisher.py robot1'),
            call({'ipAddress':'192.168.0.122'}, 'rosrun map_republisher map_republisher.py robot2')
        ])

    @patch('services.ros_utilities.terminate_mission')
    def test_terminate_mission(self, mock_terminate):
        self.robot_controls.terminate_mission('robot')
        mock_terminate.assert_called_once_with('robot')

if __name__ == '__main__':
    unittest.main()