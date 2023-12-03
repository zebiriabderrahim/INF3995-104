from bson import ObjectId
from database import database_configuration as database
from datetime import datetime
  

def fetch_missions():
    """
    Fetches a list of missions from the 'mission_history' collection in the database.

    Returns:
    - If successful, returns the list of missions retrieved from the 'mission_history' collection
      excluding the '_id' and 'map' fields.
    - If an exception occurs during the database operation, it raises the same exception.

    Note:
    - Any error during the database retrieval results in propagating the exception.
    """
    try:
        return list(database.db["mission_history"].find({}, {"_id": 0, "map": 0}))
    except Exception as e:
        raise e

def fetch_and_update_last_mission(distance):
    try:
            id = database.db["mission_history"].find_one(sort=[("_id", -1)])["_id"]
            criteria = {"_id": ObjectId(id)}
            print(criteria)
            update_operation = {"$set": {"distance": distance}}
            return database.db["mission_history"].update_one(criteria, update_operation)
    except Exception as e:
        raise e
    
def fetch_mission_map(mission_name):
    """
    Fetches the map associated with a specific mission from the 'mission_history' collection.

    Args:
    - mission_name (str): The name of the mission to retrieve the map for.

    Returns:
    - A string representing the map associated with the provided mission name.
    - If the mission name is not found or has no associated map, returns None.

    Note:
    - Any error during the database operation results in propagating the exception.
    """
    try:
        result = database.db["mission_history"].find_one({"name": mission_name}, {"_id": 0, "map": 1})
        if result:
            return result.get('map', None)
        else:
            return None 
    except Exception as e:
        raise e


def save_mission(mission):
    """
    Saves a new mission to the 'mission_history' collection in the database.

    Args:
    - mission (dict): A dictionary representing the mission details to be saved.

    Returns:
    - A dictionary containing a success message if the mission is saved successfully.

    Note:
    - It generates a unique name for the mission and sets the current date as the mission's date.
    - If any exception occurs during the database operation, it propagates the exception.
    """
    try:
        mission["date"] = datetime.now().strftime('%Y-%m-%d')
        mission["name"] = "Mission " + str(database.db["mission_history"].count_documents({}) + 1)
        database.db["mission_history"].insert_one(mission)
        return {"message": "Mission saved successfully"}
    except Exception as e:
        raise e