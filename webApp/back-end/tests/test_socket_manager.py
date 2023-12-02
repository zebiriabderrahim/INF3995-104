from unittest.mock import patch
from controllers import routes
import unittest
from unittest.mock import patch, Mock, call
from extensions import socketio
from flask import Flask, request
from services import socket_manager, ros_utilities
from classes import mission_room
import numpy as np

class SocketManagerTest(unittest.TestCase):
    def setUp(self):
        self.app = Flask(__name__)
        self.app.register_blueprint(routes.main)
        self.app.config['TESTING'] = True
        self.socketio = socketio.init_app(self.app)
        self.socket_manager = socket_manager
        self.socket_manager.ros_utilities = ros_utilities

        self.socket_manager.mission_rooms = {}
        self.socket_manager.simulated_rooms = {}
        self.robot_data = {
            "name": "Robot",
            "ipAddress": "0.0.0.0",
            "batteryLevel": 100
        }

        self.sample_data = {
            "info": {"width": 5, "height": 5},  
            "data": [
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
                -1, -1, -1, -1, -1,
                0, 0, 0, 0, 0,
                1, 0, 0, 0, 0
            ]  
        }

        self.mock_ros_class = patch('services.socket_manager.roslibpy.Ros', autospec=True)
        self.mock_ros = self.mock_ros_class.start()
        
        self.mock_topic_class = patch('services.socket_manager.roslibpy.Topic', autospec=True)
        self.mock_topic = self.mock_topic_class.start()

    def tearDown(self):
        self.socket_manager.mission_rooms = {}
        self.socket_manager.simulated_rooms = {}
        self.mock_ros_class.stop()
        self.mock_topic_class.stop()

    # @patch('time.strftime')
    # @patch('numpy.linalg.norm')
    # @patch('services.socket_service.socketio.emit')
    # def test_map_callback(self, mock_emit, mock_np, mock_time):
    #     #  testing normal functionality
    #     mock_np.return_value = 3.0
    #     mock_time.return_value = 2
    #     self.socket_manager.map_callback(self.sample_data, '192.168.0.110')

    #     mock_emit.assert_has_calls([
    #         call('map', self.sample_data['data'], room='192.168.0.110'),
    #         call('log', {'type': 'other', 'name': 'lidar', 'message': 'Obstacle en [-5.0, -4.8] à 3.0 m de robot 2', 'timestamp': 2}, room='192.168.0.110')
    #     ])


    # @patch('time.strftime')
    # @patch('numpy.linalg.norm')
    # @patch('services.socket_service.socketio.emit')
    # def test_map_callback_v2(self, mock_emit, mock_np, mock_time):
    #     # testing  np.linalg.norm(np.array(existing_position) - np.array(position_tuple)) < cluster_distance
    #     mock_np.return_value = 0.1
    #     mock_time.return_value = 2
    #     self.socket_manager.global_obstacle_dict = {(-5.0, -4.8): 3.0}
    #     self.socket_manager.map_callback(self.sample_data, '192.168.0.110')

    #     mock_emit.assert_has_calls([
    #         call('map', self.sample_data['data'], room='192.168.0.110'),
    #     ])

    # @patch('time.strftime')
    # @patch('services.socket_service.socketio.emit')
    # def test_map_callback_v3(self, mock_emit, mock_time):
    #     #  testing distance1 < distance2
    #     self.socket_manager.robot1_position = [-1, -2]
    #     mock_time.return_value = 2
    #     self.socket_manager.global_obstacle_dict = {(-5.0, -4.8): 3.0}
    #     self.socket_manager.map_callback(self.sample_data, '192.168.0.110')

    #     mock_emit.assert_has_calls([
    #         call('map', self.sample_data['data'], room='192.168.0.110'),
    #     ])

    
    @patch('services.socket_manager.sklearn.cluster.DBSCAN')
    @patch('services.socket_service.socketio.emit')
    def test_map_callback_v4(self, mock_emit, mock_dbscan):
        #  cluster_id = -1
        mock_dbscan_instance = Mock()
        mock_dbscan_instance.labels_ = np.array([0, 0, 0, -1, -1])  # Example with some points as noise
        mock_dbscan.return_value.fit = mock_dbscan_instance
        self.socket_manager.robot1_position = [-1, -2]

        self.socket_manager.map_callback(self.sample_data, '192.168.0.110')

        mock_emit.assert_has_calls([
            call('map', self.sample_data['data'], room='192.168.0.110'),
        ])


    @patch('time.strftime')
    @patch('services.socket_manager.join_room')
    @patch('services.socket_service.socketio.emit')
    def test_create_mission_room_simulation_all(self, mock_emit, mock_join_room, mock_time):
        mock_time.return_value = 0
        expected_robot = mission_room.Robot("Robots", "simulation", "(simulation)")

        with self.app.test_request_context(environ_base={'REMOTE_ADDR': '127.0.0.1'}):
            request.environ['socketio'] = socketio.server
            request.sid = 'sid'
            self.socket_manager.create_mission_room(None)
            expected_room = mission_room.MissionRoom(request.sid)
        
        mock_join_room.assert_called_once_with('simulation')
        mock_emit.assert_has_calls([
            call("createdMissionRoom", expected_room.to_dict()),
            call("log", {"type": "system", "name": "system", "message": "Mission démarrée", "timestamp": 0}, room='simulation')
        ])
        assert isinstance(self.socket_manager.simulated_rooms['simulation'].host_id, str), "Argument is not a string"
        assert self.socket_manager.simulated_rooms['simulation'].guest_id == [], "Incorrect guest_id value"
        assert self.socket_manager.simulated_rooms['simulation'].robot_info.to_dict() == expected_robot.to_dict(), "Robot has unmatched values"

        
    @patch('time.strftime')
    @patch('services.socket_manager.join_room')
    @patch('services.socket_service.socketio.emit')
    def test_create_mission_room_one_simulated(self, mock_emit, mock_join_room, mock_time):
        mock_time.return_value = 0
        expected_robot = mission_room.Robot(self.robot_data["name"], self.robot_data["ipAddress"], "(simulation)")

        with self.app.test_request_context(environ_base={'REMOTE_ADDR': '127.0.0.1'}):
            request.environ['socketio'] = socketio.server
            request.sid = 'sid'
            self.socket_manager.create_mission_room(self.robot_data, 'robot simulation')
            expected_room = mission_room.MissionRoom(request.sid, self.robot_data, None, 'simulation')
        
        mock_join_room.assert_called_once_with('0.0.0.0sim')
        mock_emit.assert_has_calls([
            call("createdMissionRoom", expected_room.to_dict()),
            call("log", {"type": "system", "name": "system", "message": "Mission démarrée", "timestamp": 0}, room='0.0.0.0sim')
        ])
        assert isinstance(self.socket_manager.simulated_rooms['0.0.0.0'].host_id, str), "Argument is not a string"
        assert self.socket_manager.simulated_rooms['0.0.0.0'].guest_id == [], "Incorrect guest_id value"
        assert self.socket_manager.simulated_rooms['0.0.0.0'].robot_info.to_dict() == expected_robot.to_dict(), "Robot has unmatched values"

    @patch('time.strftime')
    @patch('services.socket_manager.join_room')
    @patch('services.socket_service.socketio.emit')
    def test_create_mission_room_all_physical(self, mock_emit, mock_join_room, mock_time):
        mock_time.return_value = 0
        expected_robot = mission_room.Robot(self.robot_data["name"], self.robot_data["ipAddress"], "", self.robot_data["batteryLevel"])

        with self.app.test_request_context(environ_base={'REMOTE_ADDR': '127.0.0.1'}):
            request.environ['socketio'] = socketio.server
            request.sid = 'sid'
            self.socket_manager.create_mission_room([self.robot_data], 'physical')
            expected_room = mission_room.MissionRoom(request.sid, [self.robot_data], None, 'physical')
        
        mock_join_room.assert_called_once_with('physical')
        mock_emit.assert_has_calls([
            call("createdMissionRoom", expected_room.to_dict()),
            call("log", {"type": "system", "name": "system", "message": "Mission démarrée", "timestamp": 0}, room='physical')
        ])
        assert self.socket_manager.mission_rooms['physical'].to_dict() == expected_room.to_dict(), "Incorrect mission room"

    
    @patch('time.strftime')
    @patch('services.socket_manager.join_room')
    @patch('services.socket_service.socketio.emit')
    def test_create_mission_room_one_physical(self, mock_emit, mock_join_room, mock_time):
        mock_time.return_value = 0

        with self.app.test_request_context(environ_base={'REMOTE_ADDR': '127.0.0.1'}):
            request.environ['socketio'] = socketio.server
            request.sid = 'sid'
            self.socket_manager.create_mission_room(self.robot_data)
            expected_room = mission_room.MissionRoom(request.sid, self.robot_data)
        
        mock_join_room.assert_called_once_with('0.0.0.0')
        mock_emit.assert_has_calls([
            call("createdMissionRoom", expected_room.to_dict()),
            call("log", {"type": "system", "name": "system", "message": "Mission démarrée", "timestamp": 0}, room='0.0.0.0')
        ])
        assert self.socket_manager.mission_rooms['0.0.0.0'].to_dict() == expected_room.to_dict(), "Incorrect mission room"


    @patch('services.socket_manager.join_room')
    def test_create_mission_room_exception(self, mock_join_room):

        with self.app.test_request_context(environ_base={'REMOTE_ADDR': '127.0.0.1'}):
            request.environ['socketio'] = socketio.server
            request.sid = 'sid'

            mock_join_room.side_effect = Exception("")
            with patch('builtins.print') as mock_print:  
                self.socket_manager.create_mission_room(self.robot_data)
                mock_print.assert_called_once_with("An error occurred in create_mission_room function: ")

  
    @patch('services.socket_service.socketio.emit')
    @patch('services.socket_manager.close_room')
    def test_handle_stop_mission(self, mock_close_room, mock_emit):
        self.socket_manager.mission_rooms = {"0.0.0.0": mission_room.MissionRoom("host", self.robot_data)}

        with self.app.test_request_context():
            self.socket_manager.handle_stop_mission(self.robot_data)

        mock_close_room.assert_called_once_with('0.0.0.0')
        assert mock_emit.call_count == 3
        self.assertNotIn(self.robot_data["ipAddress"], self.socket_manager.mission_rooms)

    
    @patch('services.socket_service.socketio.emit')
    @patch('services.socket_manager.close_room')
    def test_handle_stop_mission_all_robots(self, mock_close_room, mock_emit):
        self.socket_manager.mission_rooms = {"physical": mission_room.MissionRoom("host", self.robot_data)}

        with self.app.test_request_context():
            self.socket_manager.handle_stop_mission(None)

        mock_close_room.assert_called_once_with('physical')
        assert mock_emit.call_count == 3
        self.assertNotIn('physical', self.socket_manager.mission_rooms)


    @patch('services.socket_service.socketio.emit')
    def test_handle_stop_mission_exception(self, mock_emit):
        self.socket_manager.mission_rooms = {"physical": mission_room.MissionRoom("host", self.robot_data)}

        with self.app.test_request_context():
            mock_emit.side_effect = Exception("")
            with patch('builtins.print') as mock_print:  
                self.socket_manager.handle_stop_mission(None)
                mock_print.assert_called_once_with("An error occurred in handle stop mission function:")

    
    @patch('time.strftime')
    @patch('services.socket_manager.close_room')
    @patch('services.socket_service.socketio.emit')
    def test_stop_simulation_all(self, mock_emit, mock_close, mock_time):
        mock_time.return_value = 0
        self.socket_manager.simulated_rooms['simulation'] = 'robot Data'
        self.socket_manager.stop_simulation(self.robot_data, 'simulation')

        mock_emit.assert_has_calls([
            call("log", {"type": "system", "name": "system", "message": "Simulation arrêtée", "timestamp": 0}, room='simulation'),
            call("hostLeftRoom", room='simulation'),
            call("roomDeleted"),
            call('stoppedSimulation', room='simulation')
        ])
        
        self.assertNotIn('simulation', self.socket_manager.simulated_rooms)
        mock_close.assert_called_once_with('simulation')

    
    @patch('time.strftime')
    @patch('services.socket_manager.close_room')
    @patch('services.socket_service.socketio.emit')
    def test_stop_simulation_one(self, mock_emit, mock_close, mock_time):
        mock_time.return_value = 0
        self.socket_manager.simulated_rooms['0.0.0.0'] = 'robot Data'
        self.socket_manager.stop_simulation(self.robot_data)

        mock_emit.assert_has_calls([
            call("log", {"type": "system", "name": "system", "message": "Simulation arrêtée", "timestamp": 0}, room='0.0.0.0sim'),
            call("hostLeftRoom", room='0.0.0.0sim'),
            call("roomDeleted"),
            call('stoppedSimulation', room='0.0.0.0sim')
        ])
        
        self.assertNotIn('0.0.0.0', self.socket_manager.simulated_rooms)
        mock_close.assert_called_once_with('0.0.0.0sim')


    @patch('services.socket_service.socketio.emit')
    def test_stop_simulation_exception(self, mock_emit):
        mock_emit.side_effect = Exception("")
        with patch('builtins.print') as mock_print:  
            self.socket_manager.stop_simulation(self.robot_data)
            mock_print.assert_called_once_with("An error occurred in stop_simulation function: ")


    @patch('services.socket_service.socketio.emit')
    def test_get_available_rooms(self, mock_emit):
        expected_room = mission_room.MissionRoom("host1", self.robot_data)
        self.socket_manager.mission_rooms = {"0.0.0.0": expected_room}
        self.socket_manager.simulated_rooms = {'0.0.0.0sim': expected_room}
        self.socket_manager.get_available_rooms()
        mock_emit.assert_called_once_with("availableRooms", {'rooms': [expected_room.to_dict()], 'simulated': [expected_room.to_dict()]})

    @patch('services.socket_service.socketio.emit')
    def test_get_available_rooms_exception(self, mock_emit):
        mock_emit.side_effect = Exception("")
        with patch('builtins.print') as mock_print:  
            self.socket_manager.get_available_rooms()
            mock_print.assert_called_once_with("An error occurred in get_available_rooms function: ")


    @patch('services.socket_manager.join_room')
    @patch('services.socket_service.socketio.emit')
    def test_view_mission_room(self, mock_emit, mock_join_room):
        self.socket_manager.mission_rooms = {'0.0.0.0': mission_room.MissionRoom("host", self.robot_data)}

        with self.app.test_request_context(environ_base={'REMOTE_ADDR': '127.0.0.1'}):
            request.environ['socketio'] = socketio.server
            request.sid = 'random_sid'
            self.socket_manager.view_mission_room(self.robot_data)
        
        mock_join_room.assert_called_once_with('0.0.0.0')
        mock_emit.assert_called_once_with("addedAsViewer", self.socket_manager.mission_rooms['0.0.0.0'].to_dict(), room='0.0.0.0')
        assert self.socket_manager.mission_rooms['0.0.0.0'].guest_id == ["random_sid"], "Guest was not added to the mission room"


    @patch('services.socket_manager.join_room')
    def test_view_mission_room_exception(self, mock_join_room):
        mock_join_room.side_effect = Exception("")
        with self.app.test_request_context(environ_base={'REMOTE_ADDR': '127.0.0.1'}):
            request.environ['socketio'] = socketio.server
            request.sid = 'random_sid'
            with patch('builtins.print') as mock_print:  
                self.socket_manager.view_mission_room(self.robot_data)
                mock_print.assert_called_once_with("An error occurred in view_mission_room function: ")
        
    @patch('services.socket_manager.odom_callback_robot')
    @patch('services.socket_manager.map_callback')
    def test_send_log(self, mock_map_callback, mock_odom_callback):
        self.socket_manager.ros_utilities.ros_connections = {'0.0.0.0': self.mock_ros.return_value}
        self.socket_manager.send_log(self.robot_data)

        args, _ = self.mock_topic.return_value.subscribe.call_args_list[0]
        args2, _ = self.mock_topic.return_value.subscribe.call_args_list[1]
        self.assertTrue(callable(args[0]))
        self.assertTrue(callable(args2[0]))

        args[0]('test message')
        args2[0]('test message')

        mock_map_callback.assert_called_once_with('test message', '0.0.0.0')
        mock_odom_callback.assert_called_once_with('test message', self.robot_data)
            
    @patch('services.socket_manager.odom_callback_robot')
    @patch('services.socket_manager.map_callback')
    def test_send_log_v2(self, mock_map_callback, mock_odom_callback):
        self.socket_manager.ros_utilities.ros_connections = {}
        self.socket_manager.send_log(self.robot_data)

        args, _ = self.mock_topic.return_value.subscribe.call_args_list[0]
        args2, _ = self.mock_topic.return_value.subscribe.call_args_list[1]
        self.assertTrue(callable(args[0]))
        self.assertTrue(callable(args2[0]))

        args[0]('test message')
        args2[0]('test message')

        self.mock_ros.return_value.run.assert_called_once()
        mock_map_callback.assert_called_once_with('test message', '0.0.0.0')
        mock_odom_callback.assert_called_once_with('test message', self.robot_data)

    def test_send_log_exception(self):
        with patch('builtins.print') as mock_print:  
            self.socket_manager.send_log({'name': 'Robot 2'})
            mock_print.assert_called_once_with("An error occurred in send_log function: log couldn't be sent, 'ipAddress'")
        

    @patch('services.socket_service.socketio.emit')
    def test_send_robot_battery(self, mock_emit):
        self.socket_manager.send_robot_battery('0.0.0.0', {'data': 100})
        mock_emit.assert_called_once_with("robotBattery", {'ipAddress': '0.0.0.0', 'batteryLevel': 100})


    @patch('time.strftime')
    @patch('services.socket_service.socketio.emit')
    def test_send_robot_battery_v2(self, mock_emit, mock_time):
        mock_time.return_value = 0
        self.socket_manager.send_robot_battery('0.0.0.0', {'data': 19})
        mock_emit.assert_has_calls([
            call("log", {"type": "system", "name": "system", "message": f"Niveau de batterie faible: 19%", "timestamp": 0}, room='0.0.0.0'),
            call("robotBattery", {'ipAddress': '0.0.0.0', 'batteryLevel': 19})
        ])


    @patch('services.socket_service.socketio.emit')
    def test_send_robot_battery_exception(self, mock_emit):
        mock_emit.side_effect = Exception("")
        with patch('builtins.print') as mock_print:  
            self.socket_manager.send_robot_battery('0.0.0.0', {'data': 100})
            mock_print.assert_called_once_with("An error occurred in send_robot_battery function: ")
        

    @patch('services.socket_manager.handle_stop_mission')
    def test_handle_disconnect_host_physical(self, mock_stop_mission):
        expected_room = mission_room.MissionRoom("host", self.robot_data)
        self.socket_manager.mission_rooms = {"0.0.0.0": expected_room}
        self.socket_manager.handle_disconnect("host")
        mock_stop_mission.assert_called_once_with(expected_room.robot_info.to_dict())

    @patch('services.socket_manager.leave_room')
    def test_handle_disconnect_guest_physical(self, mock_leave_room):
        expected_room = mission_room.MissionRoom("host", self.robot_data)
        expected_room.add_guest("guest")
        self.socket_manager.mission_rooms = {"0.0.0.0": expected_room}
        self.socket_manager.handle_disconnect("guest")
        mock_leave_room.assert_called_once_with("0.0.0.0", "guest")
        assert self.socket_manager.mission_rooms["0.0.0.0"].guest_id == [], "Guest was not disconnected"
 
    @patch('services.robot_simulation.terminate_mission_robot')
    @patch('services.socket_manager.stop_simulation')
    def test_handle_disconnect_host_simulation(self, mock_stop_sim, mock_terminate):
        expected_room = mission_room.MissionRoom('host')
        expected_room.robot_info.ip_address = 'simulation'
        self.socket_manager.simulated_rooms = {'value': expected_room}

        self.socket_manager.handle_disconnect("host")
        mock_terminate.assert_called_once()
        mock_stop_sim.assert_called_once_with(expected_room.robot_info.to_dict(), 'simulation')

    @patch('services.robot_simulation.terminate_mission_robot')
    @patch('services.socket_manager.stop_simulation')
    def test_handle_disconnect_host_simulation_v2(self, mock_stop_sim, mock_terminate):
        expected_room = mission_room.MissionRoom('host', self.robot_data, None, 'simulation')
        self.socket_manager.simulated_rooms = {'value': expected_room}

        self.socket_manager.handle_disconnect("host")
        mock_terminate.assert_called_once_with(expected_room.robot_info.to_dict())
        mock_stop_sim.assert_called_once_with(expected_room.robot_info.to_dict())

    @patch('services.socket_manager.handle_stop_mission')
    def test_handle_disconnect_exception(self, mock_stop_mission):
        expected_room = mission_room.MissionRoom("host", self.robot_data)
        self.socket_manager.mission_rooms = {"0.0.0.0": expected_room}
        mock_stop_mission.side_effect = Exception("")

        with patch('builtins.print') as mock_print:  
            self.socket_manager.handle_disconnect("host")
            mock_print.assert_called_once_with("An error occurred on handle disconnect: ")
        

    @patch('services.socket_manager.join_room')
    @patch('services.socket_service.socketio.emit')
    def test_view_all_robots(self, mock_emit, mock_join_room):
        self.socket_manager.mission_rooms = {'physical': mission_room.MissionRoom("host", self.robot_data)}

        with self.app.test_request_context(environ_base={'REMOTE_ADDR': '127.0.0.1'}):
            request.environ['socketio'] = socketio.server
            request.sid = 'random_sid'
            self.socket_manager.view_all_robots()
        
        mock_join_room.assert_called_once_with('physical')
        mock_emit.assert_called_once_with("addedAsViewer", self.socket_manager.mission_rooms['physical'].to_dict(), room='physical')
        assert self.socket_manager.mission_rooms['physical'].guest_id == ["random_sid"], "Guest was not added to the mission room"

    
    @patch('services.socket_manager.join_room')
    def test_view_all_robots_exception(self, mock_join_room):
        mock_join_room.side_effect = Exception('')
        with patch('builtins.print') as mock_print:  
            self.socket_manager.view_all_robots()
            mock_print.assert_called_once_with("An error occurred on view mission room -- all physical robots: ")
        


    
if __name__ == '__main__':
    unittest.main()
    