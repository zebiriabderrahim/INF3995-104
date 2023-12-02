import unittest
from flask import Flask
from pymongo import DESCENDING
from controllers import routes
from database import database_communication
from mongomock import MongoClient
from unittest.mock import patch
import datetime

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
        self.database_communication.database.db['mission_history'].drop()


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

        with patch.object(self.database_communication.database.db['mission_history'], 'find') as mock_find:
            mock_find.side_effect = Exception('Database error')

            with self.assertRaises(Exception) as context:
                self.database_communication.fetch_missions()

            self.assertEqual(str(context.exception), 'Database error')
            self.assertIsInstance(context.exception, Exception)

    def test_fetch_mission_map(self):
        self.database_communication.database.db['mission_history'].insert_many([
            {"_id": 0, "name": "Mission1", "map": "map1"},
            {"_id": 1, "name": "Mission2", "map": "map2"},
        ])

        mission_map = self.database_communication.fetch_mission_map('Mission1')
        self.assertEqual(mission_map, 'map1')

        mission_map = self.database_communication.fetch_mission_map('Mission2')
        self.assertEqual(mission_map, 'map2')

        mission_map = self.database_communication.fetch_mission_map('Mission3')
        self.assertEqual(mission_map, None)

        with patch.object(self.database_communication.database.db['mission_history'], 'find_one') as mock_find_one:
            mock_find_one.side_effect = Exception('Database error')

            with self.assertRaises(Exception) as context:
                self.database_communication.fetch_mission_map('Mission1')

            self.assertEqual(str(context.exception), 'Database error')
            self.assertIsInstance(context.exception, Exception)

    @patch('database.database_communication.datetime')
    def test_save_mission(self, mock_datetime):
        self.database_communication.database.db['mission_history'].insert_many([
            {"_id": 0, "name": "Mission 1", "date": "05/08/2022"},
            {"_id": 1, "name": "Mission 2", "date": "02/25/2023"},
        ])

        mock_datetime.now.return_value = datetime.datetime(2023, 11, 16)
        mission = {'name': 'Mission3', "date": "09/22/2023"}
        response = self.database_communication.save_mission(mission)

        self.assertEqual(response,  {"message": "Mission saved successfully"})
        last_inserted = self.database_communication.database.db['mission_history'].find().sort('_id', DESCENDING).limit(1)[0]
        self.assertEqual(last_inserted['name'], 'Mission 3')
        self.assertEqual(last_inserted['date'], '2023-11-16')

        with patch.object(self.database_communication.database.db['mission_history'], 'insert_one') as mock_insert_one:
            mock_insert_one.side_effect = Exception('Database error')

            with self.assertRaises(Exception) as context:
                self.database_communication.save_mission(mission)

            self.assertEqual(str(context.exception), 'Database error')
            self.assertIsInstance(context.exception, Exception)

  
if __name__ == '__main__':
    unittest.main()
