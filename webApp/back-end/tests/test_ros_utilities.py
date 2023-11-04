import unittest
from unittest.mock import patch, Mock, call
from flask import Flask
from controllers import routes
from services import ros_utilities
from functools import partial

class RosUtilities(unittest.TestCase):
    def setUp(self):
        self.app = Flask(__name__)
        self.app.register_blueprint(routes.main)
        self.app.config['TESTING'] = True
        self.client = self.app.test_client()
        self.ros_utilities = ros_utilities
        self.ros_utilities.ros_connections = {}

        self.mock_ros_class = patch('services.ros_utilities.Ros', autospec=True)
        self.mock_ros = self.mock_ros_class.start()

        self.mock_topic_class = patch('services.ros_utilities.Topic', autospec=True)
        self.mock_topic = self.mock_topic_class.start()

        self.expected_calls = [
            call({'data': 'random.launch'}),
            call({'command': "start"}),
            call({'command': "stop"})
        ]

    def tearDown(self):
        self.ros_utilities.ros_connections = {}
        self.mock_ros_class.stop()
        self.mock_topic_class.stop()

    def test_create_topic(self):
        mock_topic = self.mock_topic.return_value
        sound_topic = self.ros_utilities.create_topic(self.mock_ros.return_value, "Test_Topic")
        self.mock_topic.assert_called_once_with(self.mock_ros.return_value, "Test_Topic", 'std_msgs/String')
        mock_topic.advertise.assert_called_once()
        self.assertIs(sound_topic, mock_topic, "Topic was not created")

    @patch('services.ros_utilities.create_topic')
    def test_send_command(self, mock_create_topic):
        mock_ros = self.mock_ros.return_value
        mock_create_topic.return_value = self.mock_topic.return_value

        with self.app.app_context():
            response = self.ros_utilities.send_command("0.0.0.0", "random.launch").get_json()

        mock_ros.run.assert_called_once()
        self.mock_ros.assert_called_once_with('0.0.0.0', 9090)
        assert self.ros_utilities.ros_connections['0.0.0.0'] == mock_ros, 'Ros instance was successfully created.'
        mock_create_topic.assert_called_once_with(mock_ros, '/launch_command')
        mock_create_topic.return_value.publish.assert_called_once_with({'data': 'random.launch'})
        self.assertEqual(response["message"], "Command sent successfully")


    @patch('services.ros_utilities.create_topic')
    def test_send_command_ver2(self, mock_create_topic):
        mock_ros = self.mock_ros.return_value
        self.ros_utilities.ros_connections = {'0.0.0.1': mock_ros}
        mock_create_topic.return_value = self.mock_topic.return_value

        with self.app.app_context():
            response = self.ros_utilities.send_command("0.0.0.1", "draw_on_screen.launch").get_json()

        self.mock_ros.assert_not_called()
        mock_create_topic.assert_called_once_with(mock_ros, '/launch_command')
        mock_create_topic.return_value.publish.assert_called_once_with({'data': 'draw_on_screen.launch'})
        self.assertEqual(response["message"], "Command sent successfully")

    @patch('services.ros_utilities.time.sleep', return_value=None)
    @patch('services.ros_utilities.create_topic')
    def test_launch_mission(self, mock_create_topic, mock_sleep):    
        mock_ros = self.mock_ros.return_value
        mock_create_topic.return_value = self.mock_topic.return_value

        with self.app.app_context():
            response = self.ros_utilities.launch_mission("0.0.0.0", "random.launch").get_json()

        mock_ros.run.assert_called_once()
        self.mock_ros.assert_called_once_with('0.0.0.0', 9090)
        mock_create_topic.assert_called_with(mock_ros, '/launch_command')
        self.mock_topic.assert_called_once_with(mock_ros, '/control_command', 'launch_mission/ControlCommand')
        self.mock_topic.return_value.publish.assert_has_calls([
            call({'data': 'random.launch'}),
            call({'command': 'start'}),
            call({'command': 'stop'})
        ], any_order=False)
        mock_sleep.assert_has_calls([call(7), call(5)], any_order=False)
        self.assertEqual(response["message"], "Command sent successfully")

    @patch('services.ros_utilities.time.sleep', return_value=None)
    @patch('services.ros_utilities.create_topic')
    def test_launch_mission_2(self, mock_create_topic, mock_sleep):    
        mock_ros = self.mock_ros.return_value
        self.ros_utilities.ros_connections = {'0.0.0.1': mock_ros}
        mock_create_topic.return_value = self.mock_topic.return_value

        with self.app.app_context():
            response = self.ros_utilities.launch_mission("0.0.0.1", "random.launch").get_json()

        mock_ros.assert_not_called()
        mock_create_topic.assert_called_with(mock_ros, '/launch_command')
        self.mock_topic.assert_called_once_with(mock_ros, '/control_command', 'launch_mission/ControlCommand')
        self.mock_topic.return_value.publish.assert_has_calls([
            call({'data': 'random.launch'}),
            call({'command': 'start'}),
            call({'command': 'stop'})
        ], any_order=False)
        mock_sleep.assert_has_calls([call(7), call(5)], any_order=False)
        self.assertEqual(response["message"], "Command sent successfully")

    @patch('services.socket_manager.send_robot_battery')
    @patch('services.ros_utilities.create_topic')
    def test_subscribe_to_battery(self, mock_create_topic, mock_send_robot_battery):
        mock_ros = self.mock_ros.return_value
        mock_create_topic.return_value = self.mock_topic.return_value

        with self.app.app_context():
            response = self.ros_utilities.subscribe_to_battery("0.0.0.0").get_json()

        mock_ros.run.assert_called_once()
        self.mock_ros.assert_called_once_with('0.0.0.0', 9090)
        self.mock_topic.assert_called_once_with(mock_ros, '/battery_percentage', 'std_msgs/Float32')
        self.mock_topic.return_value.subscribe.assert_called_once()
        partial_call_args = self.mock_topic.return_value.subscribe.call_args[0][0]
        self.assertIsInstance(partial_call_args, partial)
        self.assertEqual(response["message"], "Command sent successfully")

    @patch('services.socket_manager.send_robot_battery')
    @patch('services.ros_utilities.create_topic')
    def test_subscribe_to_battery_v2(self, mock_create_topic, mock_send_robot_battery):
        mock_ros = self.mock_ros.return_value
        mock_create_topic.return_value = self.mock_topic.return_value
        self.ros_utilities.ros_connections = {'0.0.0.1': mock_ros}

        with self.app.app_context():
            response = self.ros_utilities.subscribe_to_battery("0.0.0.1").get_json()

        mock_ros.run.assert_not_called()
        self.mock_ros.assert_not_called()
        self.mock_topic.assert_called_once_with(mock_ros, '/battery_percentage', 'std_msgs/Float32')
        self.mock_topic.return_value.subscribe.assert_called_once()
        partial_call_args = self.mock_topic.return_value.subscribe.call_args[0][0]
        self.assertIsInstance(partial_call_args, partial)
        self.assertEqual(response["message"], "Command sent successfully")
        
if __name__ == '__main__':
    unittest.main()