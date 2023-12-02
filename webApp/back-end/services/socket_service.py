from flask import request
from extensions import socketio
from services import socket_manager, robot_simulation, ros_utilities, robot_controls

@socketio.on("connect")
def handle_connect():
    """
    Handles the connection event when a client connects to the server via SocketIO.

    """
    print("Client connected!")
    if len(robot_simulation.robots) == 2: socketio.emit('allSimConnected', True)
    else: socketio.emit('allSimConnected', False)  


@socketio.on("createMissionRoom")
def handle_create_mission(robot):
    """
    Handles the 'createMissionRoom' event received via SocketIO.
    This function delegates the task to 'socket_manager' to create a mission room 
    based on the provided robot information. 

    :param robot: Information about the robot involved in the mission.

    """
    socket_manager.create_mission_room(robot)
    robot_controls.launch_mission(robot)


@socketio.on("getAvailableRooms")
def get_available_rooms():
    """
    Handles the 'getAvailableRooms' event received via SocketIO.
    This function triggers 'socket_manager' to retrieve and provide information about
    available rooms where missions are actively being held.

    """
    socket_manager.get_available_rooms()


@socketio.on("viewMissionRoom")
def handle_view_mission_room(robot):
    """
    Handles the 'viewMissionRoom' event received via SocketIO.
    This function delegates the task to 'socket_manager' to view the mission 
    associated with the provided robot information.

    :param robot: Information about the robot associated with the mission room.
    
    """
    socket_manager.view_mission_room(robot)


@socketio.on("stopMission")
def handle_stop_mission(robot):
    """
    Handles the 'stopMission' event received via SocketIO.
    This function delegates the task to 'socket_manager' to handle the stopping of the 
    mission associated with the provided robot. 

    :param robot: Information about the robot associated with the mission to be stopped.

    """
    socket_manager.handle_stop_mission(robot)
    ros_utilities.terminate_mission(robot)


@socketio.on("getBatteryLevel")
def handle_get_battery_level(robot):
    """
    Handles the 'getBatteryLevel' event received via SocketIO, for the physical robots.

    :param robot: Information about the robot to retrieve battery level for.

    """
    socketio.emit("stopBatteryCall", True)
    ros_utilities.subscribe_to_battery(robot)


@socketio.on("getBatteryLevelSim")
def handle_get_battery_level(robot):
    """
    Handles the 'getBatteryLevel' event received via SocketIO, for the simulated robots.

    :param robot: Information about the robot to retrieve battery level for.

    """
    socketio.emit("stopBatteryCallSimulation", True)
    robot_simulation.subscribe_to_battery(robot)


@socketio.on("disconnect")
def handle_disconnect():
    """
    Handles the 'disconnect' event received via SocketIO.

    """
    socket_manager.handle_disconnect(request.sid)


@socketio.on("viewMissionRoomSimulation")
def handle_view_mission_room_simulation(robot):
    """
    Handles the 'viewMissionRoomSimulation' event received via SocketIO.
    This function delegates the task to 'socket_manager' to view the mission room 
    associated with the provided robot information, specifically in a simulation context.

    :param robot: Information about the robot associated with the mission room in a simulation.

    """
    socket_manager.view_mission_room(robot, True)


@socketio.on('simulateMission')
def handle_simulate_mission():
    """
    Handles the 'simulateMission' event received via SocketIO.
    This function initiates the simulation of all robots.

    """
    socket_manager.create_mission_room()
    robot_simulation.simulate_robot_mission()


@socketio.on('simulateMissionRobot')
def handle_simulate_robot_mission(robot):
    """
    Handles the 'simulateMission' event received via SocketIO.
    This function initiates the simulation of a specific robot mission.

    """
    socket_manager.create_mission_room(robot, 'robot simulation')
    robot_simulation.simulate_robot_mission(robot)


@socketio.on('stopSimulationRobot')
def handle_terminate_simulation_robot(robot):
    """
    Handles the 'stopSimulationRobot' event received via SocketIO.
    If the provided 'robot' is launched with all other robots (ipAddress == simulation),
    this function manages the termination of mission(s) for all robots within the same simulation context.
    If the robot is in it's personal simulated mission, it handles the termination of a single robot's mission.

    :param robot: Information about the robot to terminate the mission for.

    """
    if robot["ipAddress"] == 'simulation': 
        # handles the terminate mission, for both robots within the same mission
        socket_manager.get_available_rooms()

        if not robot_simulation.return_base_robots.__contains__("robot1"):
            robot_simulation.terminate_mission_robot({"name":"Robot 1"}, False)
        if not robot_simulation.return_base_robots.__contains__("robot2"):
            robot_simulation.terminate_mission_robot({"name":"Robot 2"}, False)    

        robot_simulation.terminate_mission_robot(None, True)
        socket_manager.stop_simulation(robot, 'simulation')

    else: 
        # handles the terminate mission, for one robot
        if robot_simulation.return_base_robots.__contains__(robot['name'].lower().replace(' ', '')):
            robot_simulation.terminate_mission_robot(robot, True)        
        else:
            robot_simulation.terminate_mission_robot(robot, False)
            robot_simulation.terminate_mission_robot(robot, True)
                   
        socket_manager.stop_simulation(robot)


@socketio.on('returnToBase')
def handle_return_to_base(robot):
    """
    Handles the 'returnToBase' event received via SocketIO.
    Given a 'robot' or a list of 'robots', this function triggers 'ros_utilities'
    to return the specified robot(s) to their base position(s).

    :param robot: Information about the robot(s) to return to their base position(s).

    """
    if isinstance(robot, list):
        for bot in robot:
            ros_utilities.return_robot_to_base(bot['ipAddress'])
    else:
        ros_utilities.return_robot_to_base(robot['ipAddress'])
    

@socketio.on('returnToBaseSimulation')
def handle_return_to_base_simulation(robot):
    """
    Handles the 'returnToBaseSimulation' event received via SocketIO.

    This function manages the return to base logic for robots within a simulation context.
    If the provided 'robot' exists in a mission where all existing robots are launched
    ('ipAddress' is 'simulation'), the function handles the logic for each robot independently.
    Otherwise, if the 'robot' is in its personal mission within the simulation context, it will
    be dealt with separately.

    :param robot: Information about the robot involved in returning to the base within a simulation.

    """
    if robot["ipAddress"] == 'simulation' :
        # handles return to base for both robots launched within the same mission
        if not {'robot1', 'robot2'}.issubset(robot_simulation.return_base_robots):      
            robot_simulation.terminate_mission_robot(None, False)
            robot_simulation.return_to_base()
            robot_simulation.return_base_robots.update(["robot1", "robot2"])
        # selon les tests ce code nest jamais execute car la condition dans le if sera toujours verifier
        # faudra revoir la logique, ou juste metter les lignes 117 a 120 dans le if
        # else:
        #     missing_robot = {'robot1', 'robot2'}.difference(robot_simulation.return_base_robots)
        #     robot_simulation.return_base_robots.add(missing_robot)
        #     robot_simulation.terminate_mission_robot(missing_robot, False)
        #     robot_simulation.return_to_base(missing_robot)      

    else: 
        # handles return to base for one robot
        if not robot_simulation.return_base_robots.__contains__(robot['name'].lower().replace(' ', '')):
            robot_simulation.return_base_robots.add(robot['name'].lower().replace(' ', ''))
            robot_simulation.terminate_mission_robot(robot, False)
            robot_simulation.return_to_base(robot)


@socketio.on("getLogs")
def get_logs(robots):
    """
    Handles the 'getLogs' event received via SocketIO.
    Manges the logs for the all robots, both within a physical or simulated context.

    :param robots: Information about the robot(s) to retrieve logs for.

    """
    if isinstance(robots, list):
        socket_manager.send_log(robots, True)
    else:
        socket_manager.send_log([robots])


@socketio.on('launchAllRobots')
def launch_all_robots(robots):
    """
    Handles the 'launchAllRobots' event received via SocketIO.
    Initiates the process to launch all specified 'robots'.

    :param robots: Information about the robots to be launched.

    """
    socketio.emit('allRobotsConnected', True)
    socket_manager.create_mission_room(robots, 'physical')
    robot_controls.launch_robots(robots)


@socketio.on('stopAllRobots')
def stop_all_robots(robots):
    """
    Handles the 'stopAllRobots' event received via SocketIO.
    Initiates the process to stop all specified 'robots'.

    :param robots: Information about the robots to be stopped.

    """
    socketio.emit('allRobotsConnected', False)
    socket_manager.handle_stop_mission()
    for robot in robots:
        ros_utilities.terminate_mission(robot) 


@socketio.on('viewAllRobots')
def view_all_robots():
    """
    Handles the 'viewAllRobots' event received via SocketIO.
    Initiates the process to view the mission where all robots are active. 

    """
    socket_manager.view_all_robots()


@socketio.on('setInitialPosition')
def handle_set_initial_pos(data):
    """
    Handles the 'setInitialPosition' event received via SocketIO.
    Sets the initial position for a specific robot in a simulated context.

    :param data: Contains information about the robot's name and its initial position.

    """
    robot_simulation.set_initial_position(data['name'][-1], data['data'])
    


    



