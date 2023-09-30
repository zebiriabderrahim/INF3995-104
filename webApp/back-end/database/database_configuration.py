from dotenv import dotenv_values
import pymongo
from fastapi import FastAPI
import atexit
from pymongo.errors import ServerSelectionTimeoutError, OperationFailure
from datetime import date

config = dotenv_values(".env")
app = FastAPI()

client = None

try:
    ATLAS_URI = "mongodb+srv://aymanechalh:Ali2001..@cluster0.ispathg.mongodb.net/?retryWrites=true&w=majority"
    client = pymongo.MongoClient(ATLAS_URI, serverSelectionTimeoutMS=2000)
    db = client["Projet3"]
    db.command("ismaster")
    print("Successfully connected to MongoDB")
    

except ServerSelectionTimeoutError:
    print("Error: Unable to connect to MongoDB. Check your MongoDB connection string or credentials.")
except pymongo.errors.ConnectionFailure:
    print("Error: MongoDB connection failure. Make sure MongoDB is running.")
except OperationFailure as e:
    print(f"Error: MongoDB authentication failed - {str(e)}")
except Exception as e:
    print(f"Error: An unexpected error occurred - {str(e)}")

def close_mongo_connection():
    if client is not None:
        client.close()
        print("MongoDB connection closed.")


atexit.register(close_mongo_connection)


