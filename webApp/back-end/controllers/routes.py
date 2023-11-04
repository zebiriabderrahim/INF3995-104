from flask import Blueprint, jsonify, request
from database import database_communication as db
from services import robot_controls, robot_simulation 


main = Blueprint("main", __name__)

@main.route('/missions', methods=['GET'])
def get_missions():
    try:
        return jsonify(db.fetch_missions()), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@main.route('/identify', methods=['GET'])
def identify_robot():
    try:
        robot_ip = request.args.get('ip')
        if robot_ip: return robot_controls.identify_robot(robot_ip)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@main.route('/saveMission', methods=['POST'])
def save_mission():
    try:
        mission = request.get_json()
        if mission: return jsonify(db.save_mission(mission))
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    
@main.route('/launch', methods=['GET'])
def launch_mission():
    try:
        robot_ip = request.args.get('ip')
        if robot_ip: return robot_controls.launch_mission(robot_ip)
    except Exception as e:
        return jsonify({"error": str(e)}), 500