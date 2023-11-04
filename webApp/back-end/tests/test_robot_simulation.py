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
        self.robot_simulation = robot_simulation
  
        self.mock_ros_class = patch('services.robot_simulation.roslibpy.Ros', autospec=True)
        self.mock_ros = self.mock_ros_class.start()
        
        self.mock_topic_class = patch('services.robot_simulation.roslibpy.Topic', autospec=True)
        self.mock_topic = self.mock_topic_class.start()
        
        self.mock_cmd_topic = self.mock_topic.return_value
        self.robot_simulation.client = self.mock_ros.return_value
        
        self.message = {
            'pose': {
                'pose': {
                    'position': {
                        'x': 1.0,  
                        'y': 2.0, 
                        'z': 3.0   
                    },
                    'orientation': {
                        'x': 0.0,  
                        'y': 0.0,  
                        'z': 0.0,  
                        'w': 1.0  
                    }
                }
            }
        }

        self.robot_1 = {
            "ipAddress": "192.168.0.110", 
        }

        self.robot_2 = {
            "ipAddress": "192.168.1.100", 
        }

        self.expected_calls = [
            call(self.robot_simulation.client, '/robot1/odom', 'nav_msgs/Odometry'),
            call(self.robot_simulation.client, '/robot2/odom', 'nav_msgs/Odometry')
        ]

    def tearDown(self):
        self.mock_ros_class.stop()
        self.mock_topic_class.stop()

    @patch('services.socket_service.socketio.emit')
    @patch('time.sleep')
    def test_position_callback(self, mock_sleep, mock_emit):
        self.robot_simulation.position_callback('1', self.message, 'room_1')
        mock_emit.assert_called_once_with("recieveSimRobotPos", {
            "robotId": '1',
            "position": {
                        'x': 1.0,  
                        'y': 2.0, 
                        'z': 3.0   
            }}, room='room_1')
        mock_sleep.assert_called_once_with(1)
  
    def test_terminate_mission(self):
        self.robot_simulation.client.return_value = Mock()
        self.robot_simulation.terminate_mission_robot()

        self.mock_topic.assert_has_calls(self.expected_calls, any_order=True)
        assert self.mock_cmd_topic.unadvertise.call_count == 2, "topic unadvertise was not called 2 times"
        self.robot_simulation.client.terminate.assert_called_once()

    def test_terminate_mission_v2(self):
        self.robot_simulation.client.return_value = Mock()
        self.robot_simulation.terminate_mission_robot(self.robot_1)

        self.mock_topic.assert_has_calls([self.expected_calls[0]])
        self.mock_cmd_topic.unadvertise.assert_called()
        self.robot_simulation.client.terminate.assert_called_once()

    def test_terminate_mission_v3(self):
        self.robot_simulation.client.return_value = Mock()
        self.robot_simulation.terminate_mission_robot(self.robot_2)

        self.mock_topic.assert_has_calls([self.expected_calls[1]])
        self.mock_cmd_topic.unadvertise.assert_called()
        self.robot_simulation.client.terminate.assert_called_once()
    

    @patch('services.robot_simulation.position_callback')
    @patch('services.ros_utilities.create_topic')
    def test_simulate_robot_mission_1(self, mock_create_topic, mock_callback):
        mock_create_topic.return_value = self.mock_cmd_topic

        self.robot_simulation.simulate_robot_mission('command.launch', self.robot_1)
    
        self.robot_simulation.client.run.assert_called_once()
        mock_create_topic.called_once_with(self.robot_simulation.client, 'launch_command')
        self.mock_cmd_topic.publish.assert_called_once_with({'data': 'command.launch'})
        self.mock_topic.assert_called_once_with(self.robot_simulation.client, '/robot1/odom', 'nav_msgs/Odometry')
        self.mock_topic.return_value.subscribe.assert_called_once()
        args, kwargs = self.mock_topic.return_value.subscribe.call_args
        self.assertTrue(callable(args[0]))
        lambda_function = args[0]
        lambda_function('test message')
        mock_callback.assert_called_once_with('1', 'test message', self.robot_1['ipAddress'] + 'sim')

    @patch('services.robot_simulation.position_callback')
    @patch('services.ros_utilities.create_topic')
    def test_simulate_robot_mission_2(self, mock_create_topic, mock_callback):
        mock_create_topic.return_value = self.mock_cmd_topic

        self.robot_simulation.simulate_robot_mission('command.launch', self.robot_2)
    
        self.robot_simulation.client.run.assert_called_once()
        mock_create_topic.called_once_with(self.robot_simulation.client, 'launch_command')
        self.mock_cmd_topic.publish.assert_called_once_with({'data': 'command.launch'})
        self.mock_topic.assert_called_once_with(self.robot_simulation.client, '/robot2/odom', 'nav_msgs/Odometry')
        self.mock_topic.return_value.subscribe.assert_called_once()
        args, kwargs = self.mock_topic.return_value.subscribe.call_args
        self.assertTrue(callable(args[0]))
        lambda_function = args[0]
        lambda_function('test message')
        mock_callback.assert_called_once_with('1', 'test message', self.robot_2['ipAddress'] + 'sim')

    @patch('services.robot_simulation.position_callback')
    @patch('services.ros_utilities.create_topic')
    def test_simulate_robot_mission_both(self, mock_create_topic, mock_callback):
        mock_create_topic.return_value = self.mock_cmd_topic

        self.robot_simulation.simulate_robot_mission('command.launch')
    
        self.robot_simulation.client.run.assert_called_once()
        mock_create_topic.called_once_with(self.robot_simulation.client, 'launch_command')
        self.mock_cmd_topic.publish.assert_called_once_with({'data': 'command.launch'})
        self.mock_topic.assert_has_calls(self.expected_calls)
        self.assertEqual(self.mock_topic.return_value.subscribe.call_count, 2)

        args1, _ = self.mock_topic.return_value.subscribe.call_args_list[0]
        args2, _ = self.mock_topic.return_value.subscribe.call_args_list[1]
        self.assertTrue(callable(args1[0]))
        self.assertTrue(callable(args2[0]))
        args1[0]('test message')
        args2[0]('test message')

        mock_callback.assert_has_calls([
            call("1", 'test message', 'simulation'),
            call("2", 'test message', 'simulation')
        ])


if __name__ == '__main__':
    unittest.main()
