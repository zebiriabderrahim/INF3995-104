from database import database_configuration as database



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