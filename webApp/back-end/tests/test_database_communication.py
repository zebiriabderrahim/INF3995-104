import unittest
from flask import Flask
from controllers import routes
from database import database_communication
from mongomock import MongoClient

class DatabaseCommunication(unittest.TestCase):
    def setUp(self):
        self.app = Flask(__name__)
        self.app.register_blueprint(routes.main)
        self.app.config['TESTING'] = True
        self.client = self.app.test_client()
        self.database_communication = database_communication

        self.database_communication.database.client = MongoClient()
        self.database_communication.database.db =  self.database_communication.database.client['Projet3']

    def tearDown(self):
        self.database_communication.database.client.close()
        self.database_communication.database.db['robots'].drop()
        self.database_communication.database.db['mission_history'].drop()

    def test_fetch_robots(self):
        self.database_communication.database.db['robots'].insert_many([
            {"_id": 0, "name": "Robot1", "ip": "0.0.0.0"},
            {"_id": 1, "name": "Robot2", "ip": "0.0.0.1"},
        ])

        robots = self.database_communication.fetch_robots()
        self.assertEqual(len(robots), 2)
        self.assertEqual(robots, [
            {"name": "Robot1", "ip": "0.0.0.0"},
            {"name": "Robot2", "ip": "0.0.0.1"}
        ])

    def test_fetch_missions(self):
        self.database_communication.database.db['mission_history'].insert_many([
            {"_id": 0, "name": "Mission1", "date": "05/08/2022"},
            {"_id": 1, "name": "Mission2", "date": "02/25/2023"},
        ])
         
        missions = self.database_communication.fetch_missions()
        self.assertEqual(len(missions), 2)
        self.assertEqual(missions, [
            {"name": "Mission1", "date": "05/08/2022"},
            {"name": "Mission2", "date": "02/25/2023"}
        ])


if __name__ == '__main__':
    unittest.main()
