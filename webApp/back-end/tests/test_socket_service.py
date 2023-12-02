from unittest.mock import patch, call
from services import robot_simulation
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
        self.robot_simulation = robot_simulation

        self.robot_data = {
            "name": "Robot 1",
            "ipAddress": "0.0.0.0",
            "battery": 100
        }
        
    def tearDown(self):
        self.socketio_client.disconnect()

    @patch('services.socket_service.socketio.emit')
    @patch('sys.stdout', new_callable=StringIO)
    def test_handle_connect_v1(self, mock_stdout, mock_emit):
        self.robot_simulation.robots = {'robot1', 'robot2'}
        self.socketio_client.emit("connect")
        self.assertEqual(mock_stdout.getvalue().strip(), "Client connected!")
        mock_emit.assert_called_once_with('allSimConnected', True)

    @patch('services.socket_service.socketio.emit')
    @patch('sys.stdout', new_callable=StringIO)
    def test_handle_connect_v2(self, mock_stdout, mock_emit):
        self.robot_simulation.robots = {}
        self.socketio_client.emit("connect")
        self.assertEqual(mock_stdout.getvalue().strip(), "Client connected!")
        mock_emit.assert_called_once_with('allSimConnected', False)


    @patch('services.robot_controls.launch_mission')
    @patch('services.socket_manager.create_mission_room')
    def test_handle_create_mission(self, mock_create_mission_room, mock_launch_mission):
        self.socketio_client.emit("createMissionRoom", self.robot_data)
        mock_create_mission_room.assert_called_with(self.robot_data)
        mock_launch_mission.assert_called_once_with(self.robot_data)

    @patch('services.socket_manager.get_available_rooms')
    def test_handle_get_available_rooms(self, mock_get_available_rooms):
        self.socketio_client.emit('getAvailableRooms')
        mock_get_available_rooms.assert_called_once()

    @patch('services.socket_manager.view_mission_room')
    def test_handle_view_mission_room(self, mock_view_mission_room):
        self.socketio_client.emit("viewMissionRoom", self.robot_data)
        mock_view_mission_room.assert_called_with(self.robot_data)

    @patch('services.ros_utilities.terminate_mission')
    @patch('services.socket_manager.handle_stop_mission')
    def test_handle_stop_mission(self, mock_stop_mission, mock_terminate_mission):
        self.socketio_client.emit("stopMission", self.robot_data)
        mock_stop_mission.assert_called_with(self.robot_data)
        mock_terminate_mission.assert_called_once_with(self.robot_data)

    @patch('services.socket_service.socketio.emit')
    @patch('services.ros_utilities.subscribe_to_battery')
    def test_handle_get_battery_level(self, mock_subscribe_to_battery, mock_emit):
        self.socketio_client.emit("getBatteryLevel", self.robot_data)
        mock_subscribe_to_battery.assert_called_once_with(self.robot_data)
        mock_emit.assert_called_once_with("stopBatteryCall", True)

    @patch('services.robot_simulation.subscribe_to_battery')
    @patch('services.socket_service.socketio.emit')
    def test_handle_get_battery_level_sim(self, mock_emit, mock_battery):
        self.socketio_client.emit("getBatteryLevelSim", self.robot_data)
        mock_emit.assert_called_once_with("stopBatteryCallSimulation", True)
        mock_battery.assert_called_once_with(self.robot_data)

    @patch('services.socket_manager.handle_disconnect')
    def test_handle_disconnect(self, mock_disconnect):
        with self.app.test_request_context():  
            self.socketio_client.emit("disconnect")
            mock_disconnect.assert_called()
            args, kwargs = mock_disconnect.call_args
            self.assertIsInstance(args[0], str)

    
    @patch('services.socket_manager.view_mission_room')
    def test_handle_view_mission_room_simulation(self, mock_view):
        self.socketio_client.emit('viewMissionRoomSimulation', self.robot_data)
        mock_view.assert_called_once_with(self.robot_data)


    @patch('services.robot_simulation.simulate_robot_mission')
    @patch('services.socket_manager.create_mission_room')
    def test_handle_simulate_mission(self, mock_create_mission_room, mock_simulate):
            self.socketio_client.emit('simulateMission')
            mock_create_mission_room.assert_called_once()
            mock_simulate.assert_called_once()

    @patch('services.robot_simulation.simulate_robot_mission')
    @patch('services.socket_manager.create_mission_room')
    def test_handle_simulate_robot_mission(self, mock_create_mission_room, mock_simulate):
            self.socketio_client.emit('simulateMissionRobot', self.robot_data)
            mock_create_mission_room.assert_called_once_with(self.robot_data, 'robot simulation')
            mock_simulate.assert_called_once_with(self.robot_data)

    @patch('services.robot_simulation.terminate_mission_robot')
    @patch('services.socket_manager.stop_simulation')
    def test_handle_terminate_simulation_robot1(self, mock_stop_simulate, mock_terminate):
        self.robot_simulation.return_base_robots = {'robot1'}
        self.socketio_client.emit('stopSimulationRobot', self.robot_data)
        mock_terminate.assert_called_once_with(self.robot_data, True)
        mock_stop_simulate.assert_called_once_with(self.robot_data)

    @patch('services.robot_simulation.terminate_mission_robot')
    @patch('services.socket_manager.stop_simulation')
    def test_handle_terminate_simulation_robot1_v2(self, mock_stop_simulate, mock_terminate):
        self.robot_simulation.return_base_robots = set()
        self.socketio_client.emit('stopSimulationRobot', self.robot_data)
        mock_terminate.assert_has_calls([
            call(self.robot_data, False),
            call(self.robot_data, True)
        ])
        mock_stop_simulate.assert_called_once_with(self.robot_data)

    @patch('services.robot_simulation.terminate_mission_robot')
    @patch('services.socket_manager.stop_simulation')
    @patch('services.socket_manager.get_available_rooms')
    def test_handle_terminate_simulation_robot_all(self, mock_get_rooms, mock_stop_simulate, mock_terminate):
        self.robot_simulation.return_base_robots = set ()
        self.robot_data['ipAddress'] = 'simulation'
        self.socketio_client.emit('stopSimulationRobot', self.robot_data)
        mock_get_rooms.assert_called_once()
        mock_terminate.assert_has_calls([
            call({"name":"Robot 1"}, False),
            call({"name":"Robot 2"}, False),
            call(None, True)
        ])
        mock_stop_simulate.assert_called_once_with(self.robot_data, 'simulation')


    @patch('services.ros_utilities.return_robot_to_base')
    def test_handle_return_to_base_all(self, mock_return_to_base):
        self.socketio_client.emit('returnToBase', [self.robot_data])
        mock_return_to_base.assert_called_once_with(self.robot_data['ipAddress'])


    @patch('services.ros_utilities.return_robot_to_base')
    def test_handle_return_to_base_one(self, mock_return_to_base):
        self.socketio_client.emit('returnToBase', self.robot_data)
        mock_return_to_base.assert_called_once_with(self.robot_data['ipAddress'])


    @patch('services.robot_simulation.return_to_base')
    @patch('services.robot_simulation.terminate_mission_robot')
    def test_handle_return_to_base_simulation_robot1(self, mock_terminate, mock_return_to_base):
        self.robot_simulation.return_base_robots = set ()
        self.socketio_client.emit('returnToBaseSimulation', self.robot_data)
        mock_return_to_base.assert_called_once_with(self.robot_data)
        mock_terminate.assert_called_once_with(self.robot_data, False)
        self.assertEqual(self.robot_simulation.return_base_robots, {'robot1'})

    @patch('services.robot_simulation.return_to_base')
    @patch('services.robot_simulation.terminate_mission_robot')
    def test_handle_return_to_base_simulation_both(self, mock_terminate, mock_return_to_base):
        self.robot_data['ipAddress'] = 'simulation'
        self.socketio_client.emit('returnToBaseSimulation', self.robot_data)
        mock_return_to_base.assert_called_once()
        mock_terminate.assert_called_once_with(None, False)
        self.assertEqual(self.robot_simulation.return_base_robots,  {'robot1', 'robot2'})


    # @patch('services.robot_simulation.return_to_base')
    # @patch('services.robot_simulation.terminate_mission_robot')
    # def test_handle_return_to_base_simulation_both_v2(self, mock_terminate, mock_return_to_base):
    #     self.robot_simulation.return_base_robots = {'robot2'}
    #     self.robot_data['ipAddress'] = 'simulation'
    #     self.socketio_client.emit('returnToBaseSimulation', self.robot_data)
    #     mock_return_to_base.assert_called_once_with('robot1')
    #     mock_terminate.assert_called_once_with('robot1', False)
    #     self.assertEqual(self.robot_simulation.return_base_robots,  {'robot1', 'robot2'})


    @patch('services.socket_manager.send_log')
    def test_get_logs_one(self, mock_log):
        self.socketio_client.emit('getLogs', self.robot_data)
        mock_log.assert_called_once_with([self.robot_data])

    @patch('services.socket_manager.send_log')
    def test_get_logs_all(self, mock_log):
        self.socketio_client.emit('getLogs', [self.robot_data])
        mock_log.assert_called_once_with([self.robot_data], True)

    @patch('services.socket_service.socketio.emit')
    @patch('services.robot_controls.launch_robots')
    @patch('services.socket_manager.create_mission_room')
    def test_launch_all_robots(self, mock_create_room, mock_launch_robots, mock_emit):
        self.socketio_client.emit('launchAllRobots', [self.robot_data, self.robot_data])
        mock_create_room.assert_called_once_with([self.robot_data, self.robot_data], 'physical')
        mock_launch_robots.assert_called_once_with([self.robot_data, self.robot_data])
        mock_emit.assert_called_once_with('allRobotsConnected', True)

    @patch('services.socket_service.socketio.emit')
    @patch('services.ros_utilities.terminate_mission')
    @patch('services.socket_manager.handle_stop_mission')
    def test_stop_all_robots(self, mock_stop_mission, mock_terminate, mock_emit):
        self.socketio_client.emit('stopAllRobots', [self.robot_data, self.robot_data])
        mock_stop_mission.assert_called_once()
        self.assertEqual(mock_terminate.call_count, 2)
        mock_emit.assert_called_once_with('allRobotsConnected', False)

    @patch('services.socket_manager.view_all_robots')
    def test_view_all_robots(self, mock_view):
        self.socketio_client.emit('viewAllRobots')
        mock_view.assert_called_once()

    @patch('services.robot_simulation.set_initial_position')
    def test_handle_set_initial_pos(self, mock_initial_position):
        self.socketio_client.emit('setInitialPosition', {'name': 'Robot 1', 'data': {'x': 0.0, 'y': 0.0, 'z': 0.0}})
        mock_initial_position.assert_called_once_with('1', {'x': 0.0, 'y': 0.0, 'z': 0.0})


if __name__ == '__main__':
    unittest.main()
    # to run these tests, execute python -m unittest tests.test_socket_service
