from flask import Blueprint, jsonify, request
from database import database_communication as db
from services import robot_controls, robot_update
import json

main = Blueprint("main", __name__)

@main.route('/missions', methods=['GET'])
def get_missions():
    ''' 
    retrieves the missions saved within our MongoDB database 

    '''
    try:
        return jsonify(db.fetch_missions()), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@main.route('/missionMap', methods=['GET'])
def get_mission_map():
    ''' 
    retrieves the map data from a specific mission saved within our MongoDb database

    '''
    try:
        mission_name = request.args.get('missionName')
        if mission_name: return jsonify(db.fetch_mission_map(mission_name)), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@main.route('/identify', methods=['GET'])
def identify_robot():
    ''' 
    identifies the robot currently in use 

    '''
    try:
        robot = request.args.get('robot')
        if robot: return robot_controls.identify_robot(json.loads(robot))
    except Exception as e:
        return jsonify({"error on identify robot": str(e)}), 500


@main.route('/saveMission', methods=['POST'])
def save_mission():
    ''' 
    saves a mission to the MongoDB database 

    '''
    try:
        mission = request.get_json()
        if mission: return jsonify(db.save_mission(mission))
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    

@main.route('/launch', methods=['GET'])
def launch_mission():
    ''' 
    launches a mission on a the current robot when called through an http request 

    '''
    try:
        robot = request.args.get('robot')
        if robot: return robot_controls.launch_mission(json.loads(robot))
    except Exception as e:
        return jsonify({"error on launch mission": str(e)}), 500


@main.route('/robotFiles', methods=['GET'])
def get_robot_files():
    ''' 
    extracts a specific robot's local files 
    
    '''
    try:
        password = request.args.get('password')
        if password: return jsonify(robot_update.get_robot_files(password))
    except Exception as e:
        return jsonify({"error on get robot files": str(e)}), 500


@main.route('/saveRobotFiles', methods=['POST'])
def save_robot_files():
    ''' 
    saves files on a certain robot's local workspace 
    
    '''
    try:
        password = request.args.get('password')
        data = request.get_json()
        files = data.get('files', {})
        if files and password: return jsonify(robot_update.save_robot_files(password, files))
    except Exception as e:
        return jsonify({"error on save robot files": str(e)}), 500