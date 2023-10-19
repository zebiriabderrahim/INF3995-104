import unittest
from unittest.mock import patch, Mock, call
import roslibpy
from flask import Flask
from controllers import routes
from services import robot_simulation

class RobotSimulation(unittest.TestCase):
    def setUp(self):
        self.app = Flask(__name__)
        self.app.register_blueprint(routes.main)
        self.app.config['TESTING'] = True
        self.client = self.app.test_client()
        self.robot_controls = robot_simulation
  
        self.mock_ros_class = patch('services.robot_simulation.roslibpy.Ros', autospec=True)
        self.mock_ros = self.mock_ros_class.start()
        
        self.mock_topic_class = patch('services.robot_simulation.roslibpy.Topic', autospec=True)
        self.mock_topic = self.mock_topic_class.start()
        
        self.mock_cmd_topic = self.mock_topic.return_value
        self.robot_controls.client = self.mock_ros.return_value
        
        self.msg = roslibpy.Message({
            'linear': {'x': 2.0, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 1.0, 'y': 0.0, 'z': 0.0}
        })

        self.expected_calls = [
            call(self.robot_controls.client, '/robot0/cmd_vel', 'geometry_msgs/Twist'),
            call(self.robot_controls.client, '/robot1/cmd_vel', 'geometry_msgs/Twist'),
            call(self.robot_controls.client, '/robot2/cmd_vel', 'geometry_msgs/Twist')
        ]

    def tearDown(self):
        self.mock_ros_class.stop()
        self.mock_topic_class.stop()

    @patch('time.sleep')
    def test_connect_same_topic(self, mock_time):
        self.robot_controls.connect_same_topic(2.0, self.mock_cmd_topic)
        self.mock_cmd_topic.advertise.assert_called_once()
        self.mock_cmd_topic.publish.assert_called_once_with(self.msg)
        mock_time.assert_called_once_with(1)

    @patch('services.robot_simulation.connect_same_topic')
    def test_simulate_mission(self, mock_connect_same_topic):
        self.robot_controls.simulate_mission(2.0)
        self.robot_controls.client.run.assert_called_once()
        self.mock_topic.assert_has_calls(self.expected_calls, any_order=True)
        mock_connect_same_topic.assert_has_calls([call(2.0, self.mock_cmd_topic)] * 3)

    @patch('services.robot_simulation.connect_same_topic')
    def test_terminate_mission(self, mock_connect_same_topic):
        self.robot_controls.client.return_value = Mock()
        self.robot_controls.terminate_mission(0.0)

        self.mock_topic.assert_has_calls(self.expected_calls, any_order=True)
        mock_connect_same_topic.assert_has_calls([call(0.0, self.mock_cmd_topic)] * 3)
        assert self.mock_cmd_topic.unadvertise.call_count == 3, "cmd_topic unadvertise was not called 3 times"
        self.robot_controls.client.terminate.assert_called_once()

if __name__ == '__main__':
    unittest.main()
