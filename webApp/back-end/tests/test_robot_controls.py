import unittest
from unittest.mock import patch, Mock, call
import roslibpy
from flask import Flask
from controllers import routes
from services import robot_controls 

class RobotControls(unittest.TestCase):
    def setUp(self):
        self.app = Flask(__name__)
        self.app.register_blueprint(routes.main)
        self.app.config['TESTING'] = True
        self.client = self.app.test_client()
        self.robot_controls = robot_controls

    @patch('services.robot_controls.draw_on_screen')
    def test_identify_robot1(self, mock_draw_on_screen):
        with self.app.app_context():
            mock_draw_on_screen.return_value = 'Successfully called the draw_on_screen function'
            robot1_result = self.robot_controls.identify_robot('192.168.0.122')  
            mock_draw_on_screen.assert_called_once_with('192.168.0.122') 
            assert robot1_result == 'Successfully called the draw_on_screen function', 'draw_on_screen was not called via identify(ip)'


    @patch('services.robot_controls.play_sound')
    def test_identify_robot2(self, mock_play_sound):
        mock_play_sound.return_value = 'Successfully called the play_sound function'
        robot2_result = self.robot_controls.identify_robot('192.168.0.110')
        mock_play_sound.assert_called_once_with('192.168.0.110')
        assert robot2_result == 'Successfully called the play_sound function', 'play_sound was not called via identify(ip)'

    @patch('services.ros_utilities.send_command')
    def test_draw_on_screen(self, mock_send_command):
        mock_send_command.return_value = "Successfully called the ros.send_command function"
        self.robot_controls.draw_on_screen('198.162.0.122')
        mock_send_command.assert_called_once_with('198.162.0.122', 'identification/launch/draw_screen.launch')

    @patch('services.ros_utilities.send_command')
    def test_play_sound(self, mock_send_command):
        mock_send_command.return_value = "Successfully called the ros.send_command function"
        self.robot_controls.play_sound('198.168.0.110')
        mock_send_command.assert_called_once_with('198.168.0.110', 'identification/launch/play_sound.launch')

    @patch('services.ros_utilities.launch_mission')
    def test_launch_mission(self, mock_launch_mission):
        mock_launch_mission.return_value = "Successfully called the ros.launch_mission function"
        self.robot_controls.launch_mission('198.168.0.110')
        mock_launch_mission.assert_called_once_with('198.168.0.110', 'launch_mission/src/launch_mission.launch')

if __name__ == '__main__':
    unittest.main()