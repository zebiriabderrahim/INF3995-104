from unittest.mock import patch
from controllers import routes 
from flask import Flask, jsonify
import unittest
import json

class RoutesTest(unittest.TestCase):
    def setUp(self):
        self.app = Flask(__name__)
        self.app.register_blueprint(routes.main)
        self.app.config['TESTING'] = True
        self.client = self.app.test_client()

    @patch('database.database_communication.fetch_missions')
    def test_get_missions(self, mock_db_fetch):
        sample_missions = [{"id": 1, "name": "Mission 1"}, {"id": 2, "name": "Mission 2"}]
        mock_db_fetch.return_value = sample_missions

        with self.app.app_context():
            response = self.client.get('/missions')
            self.assertEqual(response.status_code, 200)
            expected_response = jsonify(sample_missions).get_json()
            self.assertEqual(response.get_json(), expected_response)

        mock_db_fetch.side_effect = Exception('Some error message')
    
        with self.app.app_context():
            response = self.client.get('/missions')
            self.assertEqual(response.status_code, 500)
            expected_error = {"error": "Some error message"}
            self.assertEqual(response.get_json(), expected_error)

    @patch('database.database_communication.fetch_mission_map')
    def test_get_mission_map(self, mock_db_fetch_map):
        sample_map = {"id": 1, "name": "Mission 1", "map": "map1"}
        mock_db_fetch_map.return_value = sample_map

        with self.app.app_context():
            response = self.client.get('/missionMap?missionName=mission1')
            self.assertEqual(response.status_code, 200)
            expected_response = jsonify(sample_map).get_json()
            self.assertEqual(response.get_json(), expected_response)

        with self.app.app_context():
            mock_db_fetch_map.side_effect = Exception('Some error message')
            response = self.client.get('/missionMap?missionName=mission1')
            self.assertEqual(response.status_code, 500)
            expected_error = {"error": "Some error message"}
            self.assertEqual(response.get_json(), expected_error)

    @patch('database.database_communication.save_mission')
    def test_save_mission(self, mock_db_save):
        sample_mission = {"id": 1, "name": "Mission 1"}
        mock_db_save.return_value = 'successfully saved mission'

        with self.app.app_context():
            response = self.client.post('/saveMission', json=sample_mission)
            self.assertEqual(response.get_json(), 'successfully saved mission')

        with self.app.app_context():
            mock_db_save.side_effect = Exception('Some error message')
            response = self.client.post('/saveMission', json={"id": 1, "name": "Mission 1"})
            data = response.get_json()
            self.assertEqual(response.status_code, 500)
            self.assertEqual(data["error"], "Some error message")

    @patch('services.robot_controls.identify_robot')
    def test_identify_robot(self, mock_identify_robot):
        with self.app.app_context():
            mock_identify_robot.return_value = jsonify({"message": "Command sent successfully"})
            response = self.client.get('/identify?robot={"name": "robot1"}')
            data = response.get_json()
            mock_identify_robot.assert_called_with({"name": "robot1"})
            self.assertEqual(data["message"], "Command sent successfully")
        
        with self.app.app_context():
            mock_identify_robot.side_effect = Exception('Some error message')
            response = self.client.get('/identify?robot={"name": "robot1"}')
            data = response.get_json()
            self.assertEqual(response.status_code, 500)
            self.assertEqual(data["error on identify robot"], "Some error message")

    @patch('services.robot_controls.launch_mission')
    def test_launch_mission(self, mock_launch_mission):
        with self.app.app_context():
            mock_launch_mission.return_value = jsonify({"message": "Command sent successfully"})
            response = self.client.get('/launch?robot={"name": "robot2"}')
            data = response.get_json()
            mock_launch_mission.assert_called_with({"name": "robot2"})
            self.assertEqual(data["message"], "Command sent successfully")

        with self.app.app_context():
            mock_launch_mission.side_effect = Exception('Some error message')
            response = self.client.get('/launch?robot={"name": "robot2"}')
            data = response.get_json()
            self.assertEqual(response.status_code, 500)
            self.assertEqual(data["error on launch mission"], "Some error message")

    @patch('services.robot_update.get_robot_files')
    def test_get_robot_files(self, mock_get_robot_files):
        mock_get_robot_files.return_value = {'message': 'get_robot_files successful'}
        with self.app.app_context():
            response = self.client.get('/robotFiles?password=1234')
            data = response.get_json()
            mock_get_robot_files.assert_called_once_with('1234')
            self.assertEqual(data['message'], 'get_robot_files successful')

        with self.app.app_context():
            mock_get_robot_files.side_effect = Exception('Some error message')
            response = self.client.get('/robotFiles?password=1234')
            data = response.get_json()
            self.assertEqual(response.status_code, 500)
            self.assertEqual(data["error on get robot files"], "Some error message")

    @patch('services.robot_update.save_robot_files')
    def test_save_robot_files(self, mock_save_robot_files):
        mock_save_robot_files.return_value = {'message': 'save_robot_files successful'}
        with self.app.app_context():
            response = self.client.post('/saveRobotFiles?password=1234', 
                            data=json.dumps(['File1', 'File2']), 
                            content_type='application/json')
            data = response.get_json()
            mock_save_robot_files.assert_called_once_with('1234', ["File1", "File2"])
            self.assertEqual(data['message'], 'save_robot_files successful')

        with self.app.app_context():
            mock_save_robot_files.side_effect = Exception('Some error message')
            response = self.client.post('/saveRobotFiles?password=1234', 
                            data=json.dumps(['File1', 'File2']), 
                            content_type='application/json')
            data = response.get_json()
            self.assertEqual(response.status_code, 500)
            self.assertEqual(data["error on save robot files"], "Some error message")


if __name__ == '__main__':
    unittest.main()
