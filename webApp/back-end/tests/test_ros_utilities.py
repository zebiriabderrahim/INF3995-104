import unittest
from unittest.mock import patch, Mock, call
import roslibpy
from flask import Flask
from controllers import routes
from services import ros_utilities, robot_controls

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
            response = self.ros_utilities.send_command("0.0.0.0", "play_sound.launch").get_json()

        mock_ros.run.assert_called_once()
        self.mock_ros.assert_called_once_with('0.0.0.0', 9090)
        assert self.ros_utilities.ros_connections['0.0.0.0'] == mock_ros, 'Ros instance was successfully created.'
        mock_create_topic.assert_called_once_with(mock_ros, '/launch_command')
        mock_create_topic.return_value.publish.assert_called_once_with({'data': 'play_sound.launch'})
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


if __name__ == '__main__':
    unittest.main()