import unittest
from unittest.mock import patch, call
import roslibpy
from flask import Flask
from controllers import routes
from services import robot_simulation, robot_controls

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

        self.mock_goal_class = patch('services.robot_simulation.roslibpy.actionlib.Goal', autospec=True)
        self.mock_goal = self.mock_goal_class.start()

        self.mock_action_class = patch('services.robot_simulation.roslibpy.actionlib.ActionClient', autospec=True)
        self.mock_action = self.mock_action_class.start()

        self.mock_cmd_topic = self.mock_topic.return_value
        self.robot_simulation.client = self.mock_ros.return_value

        self.robot_simulation.last_processed_battery_time = 2
        self.robot_simulation.battery_value = 100
        self.robot_simulation.last_processed_time = 0
        self.robot_simulation.return_base_robots = set()
        self.robot_simulation.robots = set()
        self.robot_simulation.initial_positions = {}
        self.robot_simulation.dict_of_physical_robots = {}
        self.robot_simulation.dict_of_sim_robots = {}
        
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
            'name': 'Robot 1',
            "ipAddress": "192.168.0.110", 
        }

        self.robot_2 = {
            'name': 'Robot 2',
            "ipAddress": "192.168.0.122", 
        }


    def tearDown(self):
        self.mock_ros_class.stop()
        self.mock_topic_class.stop()
        self.mock_goal_class.stop()
        self.mock_action_class.stop()
        self.robot_simulation.last_processed_battery_time = 2
        self.robot_simulation.battery_value = 100
        self.robot_simulation.last_processed_time = 0
        self.robot_simulation.return_base_robots = set()
        self.robot_simulation.robots = set()
        self.robot_simulation.initial_positions = {}
        self.robot_simulation.dict_of_physical_robots = {}
        self.robot_simulation.dict_of_sim_robots = {}

    @patch('services.robot_simulation.battery_callback')
    def test_subscribe_to_battery(self, mock_battery_callback):
        self.robot_simulation.subscribe_to_battery(self.robot_1)

        self.mock_topic.assert_called_once_with(self.robot_simulation.client, '/robot1/battery_percentage', 'std_msgs/Float32')
        self.mock_cmd_topic.subscribe.assert_called_once()

        subscription_callback = self.mock_cmd_topic.subscribe.call_args[0][0]
        subscription_callback('test message')
        mock_battery_callback.assert_called_once_with(self.robot_1, 'test message')
        self.assertEqual(self.robot_simulation.battery_on['192.168.0.110'], 'on')

        with patch('builtins.print') as mock_print:
            self.robot_simulation.subscribe_to_battery({'ipAddress': '192.168.0.122'})
            mock_print.assert_called_once_with("An error occurred in subscribe_to_battery function: 'name'")

    @patch('time.strftime')
    @patch('services.socket_service.socketio.emit')
    def test_battery_callback(self, mock_emit, mock_time):
        mock_time.return_value = 3
        self.robot_simulation.battery_callback(self.robot_1, {'data': 20})

        self.assertTrue(self.robot_simulation.is_battery_low['192.168.0.110'])
        mock_emit.assert_has_calls([
            call('receiveBatterySim', {'robotId': '192.168.0.110', 'batteryLevel': 20}),
            call('log', {'type': 'system', 'name': 'system', 'message': 'Niveau de batterie faible: 20%', 'timestamp': 3}, room='192.168.0.110sim')
        ])

    @patch('services.socket_service.socketio.emit')
    @patch('time.time')
    def test_position_callback(self, mock_time, mock_emit):
        mock_time.return_value = 3
        self.robot_simulation.position_callback('1', self.message, 'room_1')
        mock_emit.assert_called_once_with("recieveSimRobotPos", {
            "robotId": '1',
            "position": {
                        'x': 1.0,  
                        'y': 2.0, 
                        'z': 3.0   
            }}, room='room_1')
        self.assertEqual(self.robot_simulation.last_processed_time, 3)
  

    def test_send_goal_and_wait(self):
        self.robot_simulation.send_goal_and_wait(self.mock_action, '1', True)

        self.mock_goal.assert_called_once_with(self.mock_action, {'robot_namespace': '1', 'stop': True})
        self.mock_goal.return_value.send.assert_called_once()
        self.mock_goal.return_value.wait.assert_called_once_with(20)

        self.mock_goal.return_value.wait.side_effect = Exception("Timeout occurred")
        with patch('builtins.print') as mock_print:
            self.robot_simulation.send_goal_and_wait(self.mock_action, '1', True)
            mock_print.assert_called_once_with("Goal sent to Timeout occurred timed out.")


    @patch('services.socket_manager.stop_simulation')
    @patch('services.socket_service.socketio.emit')
    def test_handle_simulation_error(self, mock_emit, mock_stop_sim):
        self.robot_simulation.handle_simulation_error(None)
        mock_emit.assert_called_once_with('rosConnectionError', room='simulation')
        mock_stop_sim.assert_called_once_with(None, 'simulation')


    @patch('services.socket_manager.stop_simulation')
    @patch('services.socket_service.socketio.emit')
    def test_handle_simulation_error_v2(self, mock_emit, mock_stop_sim):
        self.robot_simulation.handle_simulation_error(self.robot_1)
        mock_emit.assert_called_once_with('rosConnectionError', room='192.168.0.110sim')
        mock_stop_sim.assert_called_once_with(self.robot_1)


    @patch('services.socket_manager.map_callback')
    @patch('services.robot_simulation.send_goal_and_wait')
    @patch('services.robot_simulation.position_callback')
    def test_simulate_robot_mission_both(self, mock_position_callback, mock_send_goal, mock_map_callback):
        self.robot_simulation.map_topic = self.mock_cmd_topic
        self.robot_simulation.action_client = self.mock_action.return_value
        self.robot_simulation.second_action_client = self.mock_action.return_value

        self.robot_simulation.simulate_robot_mission(None)

        args1, _ = self.mock_cmd_topic.subscribe.call_args_list[0]
        args2, _ = self.mock_cmd_topic.subscribe.call_args_list[1]
        args3, _ = self.mock_cmd_topic.subscribe.call_args_list[2]
        self.assertTrue(callable(args1[0]))
        self.assertTrue(callable(args2[0]))
        self.assertTrue(callable(args3[0]))

        args1[0]('test message')
        args2[0]('test message')
        args3[0]('test message')

        mock_position_callback.assert_has_calls([
            call("2", 'test message', 'simulation'),
            call("1", 'test message', 'simulation')
        ])

        mock_send_goal.assert_has_calls([
            call(self.mock_action.return_value, 'robot1', False),
            call(self.mock_action.return_value, 'robot2', False)
        ])
        mock_map_callback.assert_called_once_with('test message', 'simulation')


    @patch('services.socket_manager.map_callback')
    @patch('services.robot_simulation.send_goal_and_wait')
    @patch('services.robot_simulation.position_callback')
    def test_simulate_robot_mission_robot1(self, mock_position_callback, mock_send_goal, mock_map_callback):
        self.robot_simulation.first_map_topic = self.mock_cmd_topic
        self.mock_cmd_topic.is_subscribed = False
        self.robot_simulation.return_base_robots.add('robot1')
        self.robot_simulation.action_client = self.mock_action.return_value

        self.robot_simulation.simulate_robot_mission(self.robot_1)

        args1, _ = self.mock_cmd_topic.subscribe.call_args_list[0]
        args2, _ = self.mock_cmd_topic.subscribe.call_args_list[1]
        self.assertTrue(callable(args1[0]))
        self.assertTrue(callable(args2[0]))

        args1[0]('test message')
        args2[0]('test message')

        self.mock_action.assert_called_once_with(self.robot_simulation.client, '/stop_resume_exploration', 'limo_gazebo_sim/StopResumeExplorationAction')
        mock_position_callback.assert_called_once_with('1', 'test message', '192.168.0.110sim')
        mock_map_callback.assert_called_once_with('test message', '192.168.0.110sim')
        mock_send_goal.assert_called_once_with(self.mock_action.return_value, 'robot1', False)
        self.assertEqual(len(self.robot_simulation.return_base_robots), 0)

    @patch('services.socket_manager.map_callback')
    @patch('services.robot_simulation.send_goal_and_wait')
    @patch('services.robot_simulation.position_callback')
    def test_simulate_robot_mission_robot2(self, mock_position_callback, mock_send_goal, mock_map_callback):
        self.robot_simulation.second_map_topic = self.mock_cmd_topic
        self.mock_cmd_topic.is_subscribed = False
        self.robot_simulation.second_action_client = self.mock_action.return_value

        self.robot_simulation.simulate_robot_mission(self.robot_2)

        args1, _ = self.mock_cmd_topic.subscribe.call_args_list[0]
        args2, _ = self.mock_cmd_topic.subscribe.call_args_list[1]
        self.assertTrue(callable(args1[0]))
        self.assertTrue(callable(args2[0]))

        args1[0]('test message')
        args2[0]('test message')

        self.mock_action.assert_called_once_with(self.robot_simulation.client, '/stop_resume_exploration', 'limo_gazebo_sim/StopResumeExplorationAction')
        mock_position_callback.assert_called_once_with('2', 'test message', '192.168.0.122sim')
        mock_map_callback.assert_called_once_with('test message', '192.168.0.122sim')
        mock_send_goal.assert_called_once_with(self.mock_action.return_value, 'robot2', False)
        self.assertEqual(len(self.robot_simulation.return_base_robots), 0)


    @patch('services.robot_simulation.handle_simulation_error')
    def test_simulate_robot_mission_exception(self, mock_error):
        with patch('builtins.print') as mock_print:
            self.robot_simulation.simulate_robot_mission({'ipAddress':'0.0.0.0'})
            mock_print.assert_called_once_with("An error occurred in simulate_robot_mission function: 'name'")
            mock_error.assert_called_once_with({'ipAddress':'0.0.0.0'})


    @patch('roslibpy.Time.now')
    def test_send_return_to_base_goal(self, mock_time):
        mock_time.return_value = 2
        expected_message = {
            'target_pose': {
                'header': {
                    'frame_id': 'map',
                    'stamp': 2
                },
                'pose': {
                    'position': {'x': 1.0, 'y': 2.0, 'z': 3.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
            }
        }
        self.robot_simulation.send_return_to_base_goal('robot1', {'x': 1.0, 'y': 2.0, 'z': 3.0})

        self.mock_action.assert_called_once_with(self.robot_simulation.client, '/robot1/move_base', 'move_base_msgs/MoveBaseAction')
        self.mock_goal.assert_called_once_with(self.mock_action.return_value, roslibpy.Message(expected_message))
        self.mock_goal.return_value.send.assert_called_once()


    def test_send_return_to_base_goal_exception(self):
        with patch('builtins.print') as mock_print:
            self.mock_goal.return_value.send.side_effect = Exception('')
            self.robot_simulation.send_return_to_base_goal('robot1', {'x': 1.0, 'y': 2.0, 'z': 3.0})
            mock_print.assert_called_once_with("An error occurred in send_return_to_base_goal function: ")


    @patch('services.socket_service.socketio.emit')
    def test_set_initial_position(self, mock_emit):
        self.robot_simulation.client.is_connected = False
        self.robot_simulation.robots.add(2)
        self.robot_simulation.set_initial_position('1', {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0})

        self.robot_simulation.client.run.assert_called_once()
        self.mock_action.assert_called_once_with(self.robot_simulation.client, f"/launch_robot", 'limo_gazebo_sim/LaunchRobotAction')
        self.mock_goal.assert_called_once_with(self.mock_action.return_value, {
            'ns': 1, 
            'x': 0.0, 
            'y': 0.0, 
            'z': 0.0, 
            'yaw': 0.0
        })
        self.mock_goal.return_value.send.assert_called_once()
        self.mock_goal.return_value.wait.assert_called_once_with(10)
        self.assertTrue('robot1' in self.robot_simulation.initial_positions)
        mock_emit.assert_called_once_with('allSimConnected', True)


    def test_set_initial_position_exception(self):
        with patch('builtins.print') as mock_print:
            self.robot_simulation.set_initial_position('2', {'w': 0.0})
            mock_print.assert_called_once_with("An error occurred in set_initial_position function: 'x'")


    @patch('services.robot_simulation.send_return_to_base_goal')
    def test_return_to_base_all(self, mock_return_to_base):
        self.robot_simulation.initial_positions = {'robot1': 'pos1', 'robot2': 'pos2'}
        self.robot_simulation.return_to_base(None)
        mock_return_to_base.assert_has_calls([
            call("robot1", 'pos1'),
            call("robot2", 'pos2')
        ])


    @patch('services.robot_simulation.send_return_to_base_goal')
    def test_return_to_base_robot1(self, mock_send_return_to_base):
        self.robot_simulation.initial_positions = {'robot1': 'pos1'}
        self.robot_simulation.return_to_base(self.robot_1)
        mock_send_return_to_base.assert_called_once_with('robot1', 'pos1')


    @patch('services.robot_simulation.send_return_to_base_goal')
    def test_return_to_base_robot2(self, mock_send_return_to_base):
        self.robot_simulation.initial_positions = {'robot2': 'pos2'}
        self.robot_simulation.return_to_base(self.robot_2)
        mock_send_return_to_base.assert_called_once_with('robot2', 'pos2')


    def test_return_to_base_exception(self):
        with patch('builtins.print') as mock_print:
            self.robot_simulation.return_to_base({'ipAddress': '0.0.0.0'})
            mock_print.assert_called_once_with("An error occurred in return_to_base function: 'name'")


    @patch('services.socket_service.socketio.close_room')
    @patch('services.socket_service.socketio.emit')
    def test_distance_callback_physical(self, mock_emit, mock_close_room):
        self.robot_simulation.robot_controls = robot_controls
        self.robot_simulation.robot_controls.are_two_physical_launched = True
        self.robot_simulation.dict_of_physical_robots['Robot 2'] = 8.0
        self.robot_simulation.distance_callback('Robot 1', {'data': 6.0}, True)

        mock_emit.assert_called_once_with("receiveDistanceSim", f"Le robot 1 a parcouru 6.0 m et le robot 2 a parcouru 8.0 m en exploration physique", room='physical')
        self.assertEqual(self.robot_simulation.dict_of_physical_robots, {})
        self.assertFalse(self.robot_simulation.robot_controls.are_two_physical_launched)
        mock_close_room.assert_called_once_with('physical')


    @patch('services.socket_service.socketio.close_room')
    @patch('services.socket_service.socketio.emit')
    def test_distance_callback_physical_robot1(self, mock_emit, mock_close_room):
        self.robot_simulation.robot_controls = robot_controls
        self.robot_simulation.robot_controls.are_two_physical_launched = False
        self.robot_simulation.distance_callback('Robot 1', {'data': 6.0}, True)
        mock_emit.assert_called_once_with("receiveDistanceSim", f"Le robot  1 a parcouru 6.0 m en exploration physique", room='192.168.0.110')
        mock_close_room.assert_called_once_with('192.168.0.110')


    @patch('services.socket_service.socketio.close_room')
    @patch('services.socket_service.socketio.emit')
    def test_distance_callback_physical_robot2(self, mock_emit, mock_close_room):
        self.robot_simulation.robot_controls = robot_controls
        self.robot_simulation.robot_controls.are_two_physical_launched = False
        self.robot_simulation.distance_callback('Robot 2', {'data': 6.0}, True)
        mock_emit.assert_called_once_with("receiveDistanceSim", f"Le robot  2 a parcouru 6.0 m en exploration physique", room='192.168.0.122')
        mock_close_room.assert_called_once_with('192.168.0.122')


    @patch('services.socket_service.socketio.emit')
    def test_distance_callback_sim_all(self, mock_emit):
        self.robot_simulation.are_two_robot_connected = True
        self.robot_simulation.dict_of_sim_robots = {'robot1': 4.0}

        self.robot_simulation.distance_callback('robot2', {'total_distance': 6.0}, False)
        self.assertEqual(self.robot_simulation.simulated_robot_distance, "Le robot 1 a parcouru 4.0 m et le robot 2 a parcouru 6.0 m en simulation")
        self.assertEqual(self.robot_simulation.dict_of_sim_robots, {})
        self.assertFalse(self.robot_simulation.are_two_robot_connected)


    def test_distance_callback_sim_robot1(self):
        self.robot_simulation.distance_callback('robot1', {'total_distance': 6.0}, False)
        self.assertEqual(self.robot_simulation.simulated_robot_distance, f"Le robot 1 a parcouru 6.0 m en simulation")


    @patch('services.socket_service.socketio.emit')
    def test_distance_callback_exception(self, mock_emit):
        self.robot_simulation.robot_controls = robot_controls
        self.robot_simulation.robot_controls.are_two_physical_launched = False
        mock_emit.side_effect = Exception('')
        with patch('builtins.print') as mock_print:
            self.robot_simulation.distance_callback('Robot 1', {'data': 3.0} , True)
            mock_print.assert_called_once_with("An error occurred in distance_callback function: ")


    @patch('services.robot_simulation.distance_callback')
    def test_terminate_mission_robot_both(self, _):
        self.robot_simulation.map_topic = self.mock_cmd_topic
        self.mock_cmd_topic.is_subscribed = True
        self.robot_simulation.terminate_mission_robot(None, True)

        self.mock_action.assert_has_calls([
            call(self.robot_simulation.client, 'robot1/distance_query', 'limo_gazebo_sim/DistanceQueryAction'),
            call(self.robot_simulation.client, 'robot2/distance_query', 'limo_gazebo_sim/DistanceQueryAction')
        ])

        self.assertEqual(self.mock_goal.return_value.send.call_count, 2)
        self.assertEqual(self.mock_goal.return_value.on.call_count, 2)
        self.assertEqual(self.mock_cmd_topic.unsubscribe.call_count, 3)


    @patch('services.robot_simulation.send_goal_and_wait')
    def test_terminate_mission_robot_2(self, mock_send_goal_and_wait):
        self.robot_simulation.second_action_client = self.mock_action.return_value
        self.robot_simulation.terminate_mission_robot(self.robot_2, False)  
        mock_send_goal_and_wait.assert_called_once_with(self.mock_action.return_value, 'robot2', True)


    @patch('services.robot_simulation.send_goal_and_wait')
    def test_terminate_mission_robot1(self, mock_send_goal_and_wait):
        self.robot_simulation.action_client = self.mock_action.return_value
        self.robot_simulation.terminate_mission_robot(self.robot_1, False)  
        mock_send_goal_and_wait.assert_called_once_with(self.mock_action.return_value, 'robot1', True)


    @patch('services.robot_simulation.distance_callback')
    @patch('services.robot_simulation.send_goal_and_wait')
    def test_terminate_mission_robot_1_v2(self, __, _):
        self.robot_simulation.map_topic = self.robot_simulation.second_map_topic = self.mock_cmd_topic
        self.mock_cmd_topic.is_subscribed = True
        self.robot_simulation.terminate_mission_robot(self.robot_1, True) 
        self.assertEqual(self.mock_cmd_topic.unsubscribe.call_count, 2)


    @patch('services.robot_simulation.send_goal_and_wait')
    def test_terminate_mission_robot_exception(self, mock_send_goal_and_wait):
        mock_send_goal_and_wait.side_effect = Exception('')
        with patch('builtins.print') as mock_print:
            self.robot_simulation.terminate_mission_robot(self.robot_1)
            mock_print.assert_called_once_with("An error occurred in get_distance_and_stop function: ")

        with patch('builtins.print') as mock_print:
            self.robot_simulation.terminate_mission_robot({'ipAddress': '0.0.0.0'})
            mock_print.assert_called_once_with("An error occurred in terminate_mission_robot function: 'name'")


if __name__ == '__main__':
    unittest.main()
