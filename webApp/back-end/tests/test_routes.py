from unittest.mock import patch
from controllers import routes 
from flask import Flask, jsonify
import unittest
from services import robot_controls, robot_simulation
from database import database_communication

class RoutesTest(unittest.TestCase):
    def setUp(self):
        self.app = Flask(__name__)
        self.app.register_blueprint(routes.main)
        self.app.config['TESTING'] = True
        self.client = self.app.test_client()

    @patch('services.robot_simulation.simulate_mission')
    def test_simulate_mission(self, mock_simulate_mission):
        response = self.client.get('/simulate')
        mock_simulate_mission.assert_called_with(0.1)
        data = response.get_json()
        self.assertEqual(response.status_code, 200)
        self.assertEqual(data["message"], "Simulation successful")

    @patch('services.robot_simulation.simulate_mission')
    def test_terminate_simulation(self, mock_terminate_mission):
        response = self.client.get('/terminateSim')
        mock_terminate_mission.assert_called_with(0.0)
        data = response.get_json()
        self.assertEqual(response.status_code, 200)
        self.assertEqual(data["message"], "Termination successful")

    @patch('database.database_communication.fetch_missions')
    def test_get_missions(self, mock_db_fetch):
        sample_missions = [{"id": 1, "name": "Mission 1"}, {"id": 2, "name": "Mission 2"}]
        mock_db_fetch.return_value = sample_missions

        with self.app.app_context():
            response = self.client.get('/missions')
            self.assertEqual(response.status_code, 200)
            expected_response = jsonify(sample_missions).get_json()
            self.assertEqual(response.get_json(), expected_response)

    @patch('services.robot_controls.identify_robot')
    def test_identify_robot(self, mock_identify_robot):
        with self.app.app_context():
            mock_identify_robot.return_value = jsonify({"message": "Command sent successfully"})
            response = self.client.get('/identify?ip=1')
            data = response.get_json()
            mock_identify_robot.assert_called_with('1')
            self.assertEqual(response.status_code, 200)
            self.assertEqual(data["message"], "Command sent successfully")


if __name__ == '__main__':
    unittest.main()
    # to run these tests, execute python -m unittest tests.test_route
    # the test for get_robots was not written since we dont really use it in the frontend --may have to remove the function altogether
