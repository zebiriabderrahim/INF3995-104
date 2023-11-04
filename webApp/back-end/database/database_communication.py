from database import database_configuration as database
from datetime import datetime


def fetch_robots(): 
    try:
        return list(database.db["robots"].find({}, {"_id": 0, "name": 1, "ip": 1}))
    except Exception as e:
        raise e 
    
def fetch_missions():
    try:
        return list(database.db["mission_history"].find({}, {"_id": 0}))
    except Exception as e:
        raise e 

def save_mission(mission):
    try:
        #for now only the logs are saved and basic info
        mission["date"] = datetime.now().strftime('%Y-%m-%d')
        mission["name"] = "Mission " + str(database.db["mission_history"].count_documents({}) + 1)
        database.db["mission_history"].insert_one(mission)
        return {"message": "Mission saved successfully"}
    except Exception as e:
        raise e