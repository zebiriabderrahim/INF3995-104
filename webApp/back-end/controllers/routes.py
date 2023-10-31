from flask import Blueprint, jsonify, request
from database import database_communication as db
from services import robot_controls, robot_simulation 


main = Blueprint("main", __name__)

@main.route('/simulate', methods=['GET'])
def simulate_mission():
    try:
        robot_simulation.simulate_mission(0.1)
        return jsonify({"message": "Simulation successful"}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    
@main.route('/terminateSim', methods=['GET'])
def terminate_simulation():
    try:
        robot_simulation.simulate_mission(0.0)
        return jsonify({"message": "Termination successful"}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@main.route('/robots', methods=['GET'])
def get_robots():
    try:
        return jsonify(db.fetch_robots())
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    
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
    
@main.route('/launch', methods=['GET'])
def launch_mission():
    try:
        robot_ip = request.args.get('ip')
        if robot_ip: return robot_controls.launch_mission(robot_ip)
    except Exception as e:
        return jsonify({"error": str(e)}), 500