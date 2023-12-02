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


    def tearDown(self):
        self.ros_utilities.ros_connections = {}
        self.mock_ros_class.stop()
        self.mock_topic_class.stop()


    @patch('services.ros_utilities.execute_command')
    @patch('services.ros_utilities.create_ros')
    def test_terminate_mission(self, mock_create_ros, mock_execute_command):
        mock_ros_client = self.mock_ros.return_value
        mock_create_ros.return_value = mock_ros_client
        self.ros_utilities.terminate_mission({'ipAddress': '192.168.0.110'})
        self.assertEqual(self.mock_topic.call_count, 2)
        self.assertEqual(self.mock_topic.return_value.unsubscribe.call_count, 2)
        mock_execute_command.assert_called_once_with({'ipAddress': '192.168.0.110'}, "rosrun stop_mission stop_commander.py")


    @patch('services.ros_utilities.execute_command')
    def test_return_robot_to_base(self, mock_execute_command):
        self.ros_utilities.return_robot_to_base('192.168.0.110')
        mock_execute_command.assert_called_once_with({"ipAddress": '192.168.0.110'}, "rosrun return_base return_base.py")

    @patch('services.ros_utilities.execute_command')
    def test_return_robot_to_base_exception(self, mock_execute_command):
        mock_execute_command.side_effect = Exception('')
        with self.assertRaises(Exception) as context:
            self.ros_utilities.return_robot_to_base('192.168.0.110')
            self.assertEqual(str(context.exception), "error on return robot to base")
            self.assertIsInstance(context.exception, Exception)

    @patch('services.socket_manager.send_robot_battery')
    @patch('services.ros_utilities.execute_command')
    @patch('services.ros_utilities.create_ros')
    def test_subscribe_to_battery(self, mock_create_ros, mock_execute_command, mock_send_robot_battery):
        mock_ros_client = self.mock_ros.return_value
        mock_create_ros.return_value = mock_ros_client

        with self.app.app_context():
            response = self.ros_utilities.subscribe_to_battery({'ipAddress': '192.168.0.110'}).get_json()

        mock_create_ros.assert_called_once_with('192.168.0.110')
        mock_execute_command.assert_called_once_with({'ipAddress': '192.168.0.110'}, "roslaunch battery battery.launch")
        self.mock_topic.assert_called_once_with(mock_ros_client, '/battery_percentage', 'std_msgs/Float32')
        self.mock_topic.return_value.subscribe.assert_called_once()
        partial_call_args = self.mock_topic.return_value.subscribe.call_args[0][0]
        self.assertIsInstance(partial_call_args, partial)
        self.assertEqual(response["message"], "Command sent successfully")

    @patch('services.ros_utilities.create_ros')
    def test_subscibe_to_battery(self, mock_create_ros):
        mock_create_ros.side_effect = Exception('')
        with self.assertRaises(Exception) as context:
            self.ros_utilities.subscribe_to_battery({'ipAddress': '192.168.0.110'})
            self.assertEqual(str(context.exception), "error on subscribe to battery")
            self.assertIsInstance(context.exception, Exception)

    @patch('services.ros_utilities.create_ros')
    def test_execute_command(self, mock_create_ros):
        mock_ros_client = self.mock_ros.return_value
        mock_create_ros.return_value = mock_ros_client
        with self.app.app_context():
            response = self.ros_utilities.execute_command({'ipAddress': '192.168.0.110'}, 'command.launch').get_json()

        mock_create_ros.assert_called_once_with('192.168.0.110')
        self.mock_topic.assert_called_once_with(mock_ros_client, "/launch_command", "std_msgs/String")
        self.mock_topic.return_value.publish.assert_called_once_with({"data": 'command.launch'})
        self.assertEqual(response["message"], "Command sent successfully")
        
    @patch('services.ros_utilities.create_ros')
    def test_execute_command_exception(self, mock_create_ros):
        mock_create_ros.side_effect = Exception('')
        with self.assertRaises(Exception) as context:
            self.ros_utilities.execute_command({'ipAddress': '192.168.0.110'}, 'run.launch')
            self.assertEqual(str(context.exception), "error on execute command")
            self.assertIsInstance(context.exception, Exception)

    def test_create_ros_if(self):
        mock_ros_client = self.mock_ros.return_value
        self.ros_utilities.ros_connections['192.168.0.110'] = mock_ros_client
        object = self.ros_utilities.create_ros('192.168.0.110')
        mock_ros_client.run.assert_called_once()
        self.assertEqual(object, mock_ros_client)

    def test_create_ros_else(self):
        self.ros_utilities.create_ros('192.168.0.110')
        self.mock_ros.assert_called_once_with('192.168.0.110', 9090)
        self.assertEqual(self.ros_utilities.ros_connections, {'192.168.0.110': self.mock_ros.return_value})
        self.mock_ros.return_value.run.assert_called_once()

    def test_create_topic(self):
        topic = self.ros_utilities.create_topic(self.mock_ros.return_value, "Test_Topic")
        self.mock_topic.assert_called_once_with(self.mock_ros.return_value, "Test_Topic", 'std_msgs/String')
        self.mock_topic.return_value.advertise.assert_called_once()
        self.assertIs(topic, self.mock_topic.return_value, "Topic was not created")

   
if __name__ == '__main__':
    unittest.main()