from math import dist
import time
from turtle import distance
import roslibpy
import roslibpy.actionlib
import numpy as np
from scipy.ndimage import zoom
from services import socket_service, socket_manager, robot_controls

# client = roslibpy.Ros(host='192.168.64.12', port=9090)
client = roslibpy.Ros(host='192.168.125.245', port=9090)
map_topic = roslibpy.Topic(client, '/map', 'nav_msgs/OccupancyGrid')
second_map_topic= roslibpy.Topic(client, '/map', 'nav_msgs/OccupancyGrid')
last_processed_time = 0
last_processed_battery_time = 0
initial_positions = {}
return_base_robots = set()
robots = set()
are_two_robot_connected = False
dict_of_sim_robots = {}
dict_of_physical_robots = {}
connected_robots = 0
action_client = None
second_action_client = None


def subscribe_to_battery(robot):
    """
    Sets up a subscription to the battery status topic of a given robot.

    This function creates a ROS topic subscription to monitor battery status updates for a specific robot.
    It takes a dictionary ('robot') containing information about the robot to which the subscription is directed.

    Upon receiving battery status updates, it invokes the 'battery_callback' function to handle the messages.
    The callback function emits the battery level data to a designated socket room, ensuring clients receive updates
    no more frequently than once per second.

    :param robot: A dictionary containing information about the robot, including its name, IP address, or other relevant details.
    """
    try:
        roslibpy.Topic(client, f"/{robot['name'].lower().replace(' ', '')}/battery/status", 'sensor_msgs/BatteryState').subscribe(lambda message: battery_callback(robot, message))
    except Exception as e:
        print(f"An error occurred in subscribe_to_battery function: {str(e)}")
   
        
def battery_callback(robot, message):
    """
    Callback function that is invoked when a battery status message is received from ROS.

    It handles incoming battery status updates for a specific robot and emits the battery level data
    to a designated socket room, ensuring that clients receive updates no more frequently than once per second.

    This function uses a global variable 'last_processed_battery_time' to throttle the rate at which
    messages are processed and emitted, preventing excessive updates and improving efficiency.

    :param robot: A dictionary containing information about the robot.
    :param message: A dictionary containing the battery percentage.
    """
    global last_processed_battery_time
    
    current_time = time.time()
    if current_time - last_processed_battery_time >= 1: 
        if message['percentage'] is not None: socket_service.socketio.emit("receiveBatterySim", {"robotId": str(robot["ipAddress"]), "batteryLevel": round(message['percentage'])})
        last_processed_battery_time = current_time  
        
    
def position_callback(robot_id, message, room_id):
    """
    Callback function that is invoked when a position message is received from ROS. 
    It emits the position data to a specific socket room at most once per second.

    This function uses a global variable 'last_processed_time' to throttle the rate 
    at which messages are processed and emitted, ensuring that clients receive updates 
    no more frequently than once per second.

    :param robot_id: The ID of the robot from which the position is being reported.
    :param message: The message received from ROS with the robot's position.
    :param room_id: The socket room identifier to which the position should be emitted.
    """
    global last_processed_time
    current_time = time.time()

    if current_time - last_processed_time >= 1:  
        socket_service.socketio.emit("recieveSimRobotPos",{"robotId": str(robot_id), "position": message['pose']['pose']['position']}, room=room_id)
        last_processed_time = current_time  


def send_goal_and_wait(action_client, namespace, stop):
    """
    Sends a goal to the specified namespace of the action client and waits for a result.

    :param action_client: The action client through which to send the goal.
    :param namespace: The namespace of the robot to which the goal is sent.
    :param stop: A boolean value to start or stop the robot.
    """
    goal = roslibpy.actionlib.Goal(action_client, {'robot_namespace': namespace, 'stop': stop})
    goal.send()
    try:
        goal.wait(5)
    except Exception as e:
        print(f"Goal sent to {e} timed out.")
   
   
def handle_simulation_error(robot):
    """
    Handles errors that occur during the simulation by logging the error,
    notifying via socket, and stopping the simulation.

    :param e: The exception that occurred.
    :param robot: The robot dictionary containing the robot's information, or None if not applicable.
    """
    if robot is None: 
        room = 'simulation'
        socket_service.socketio.emit('rosConnectionError', room=room)
        socket_manager.stop_simulation(robot, 'simulation')
    else: 
        room = str(robot["ipAddress"] + 'sim')
        socket_service.socketio.emit('rosConnectionError', room=room)
        socket_manager.stop_simulation(robot)


def simulate_robot_mission(robot=None):
    """
    Simulates the mission for a robot in a simulated environment.
    
    The function manages different scenarios based on whether a single robot or both robots are involved in the simulation.
    For a single robot, it subscribes to its odom topic, manages goal sending, and handles map callbacks.
    For both robots, it establishes subscriptions to their respective odom topics, sends exploration goals to both robots,
    and subscribes to map topics for socket callbacks.

    :param robot: A dictionary containing information about the robot, including its name, IP address, or other relevant details.
    """
    global return_base_robots
    global are_two_robot_connected
    global action_client
    global second_action_client

    try:                 
        if robot is None or robot['name'] == "Robot 1":
            action_client = roslibpy.actionlib.ActionClient(client, '/stop_resume_exploration', 'limo_gazebo_sim/StopResumeExplorationAction')

        if robot is None:
            are_two_robot_connected = True
            return_base_robots.clear()
            roslibpy.Topic(client, '/robot2/odom', 'nav_msgs/Odometry').subscribe(lambda message: position_callback("2", message, 'simulation'))
            roslibpy.Topic(client, '/robot1/odom', 'nav_msgs/Odometry').subscribe(lambda message: position_callback("1", message, 'simulation'))
            for i in [1, 2]:
                send_goal_and_wait(action_client, f"robot{i}", False)
            map_topic.subscribe(lambda message: socket_manager.map_callback(message, 'simulation')) 

        else:
            roslibpy.Topic(client, f"{robot['name'].lower().replace(' ', '')}/odom", 'nav_msgs/Odometry').subscribe(lambda message: position_callback(robot["name"][-1], message, str(robot["ipAddress"] + 'sim')))

            if robot['name'] == "Robot 1" and not map_topic.is_subscribed: 
                map_topic.subscribe(lambda message: socket_manager.map_callback(message, str(robot["ipAddress"] + 'sim')))
                send_goal_and_wait(action_client, f"{robot['name'].lower().replace(' ', '')}", False)      
            elif robot['name'] == "Robot 2" and not second_map_topic.is_subscribed:
                second_action_client = roslibpy.actionlib.ActionClient(client, '/stop_resume_exploration', 'limo_gazebo_sim/StopResumeExplorationAction')
                send_goal_and_wait(second_action_client, f"{robot['name'].lower().replace(' ', '')}", False)                    
                second_map_topic.subscribe(lambda message: socket_manager.map_callback(message, str(robot["ipAddress"] + 'sim')))

            if return_base_robots.__contains__(robot['name'].lower().replace(' ', '')): return_base_robots.remove(robot['name'].lower().replace(' ', ''))

    except Exception as e:
        print(f"An error occurred in simulate_robot_mission function: {str(e)}")
        if robot is not None: handle_simulation_error(robot)


def send_return_to_base_goal(robot_name, initial_pos):
    """
    Sends a return-to-base goal to a specific robot using ROS MoveBaseAction.

    This function constructs and sends a return-to-base goal to a robot specified by 'robot_name'
    utilizing the ROS MoveBaseAction. The goal is formed with a specific initial position provided.

    :param robot_name: The name or identifier of the robot receiving the return-to-base goal.
    :param initial_pos: A dictionary specifying the initial position coordinates.
    """
    try:
        action_client = roslibpy.actionlib.ActionClient(client, f"/{robot_name}/move_base", 'move_base_msgs/MoveBaseAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({
            'target_pose': {
                'header': {
                    'frame_id': 'map',
                    'stamp': roslibpy.Time.now()
                },
                'pose': {
                    'position': {'x': initial_pos['x'], 'y': initial_pos['y'], 'z': initial_pos['z']},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
            }
        }))
        goal.send()
    except Exception as e:
        print(f"An error occurred in send_return_to_base_goal function: {str(e)}")


def set_initial_position(robot_id, position):
    """
    Sets the initial position of a robot in the simulation environment.

    This function establishes the initial position for a robot identified by 'robot_id'
    within the simulation environment. It uses ROS to send a launch_robot action goal 
    containing the specified position information.

    :param robot_id: The identifier of the robot for setting its initial position.
    :param position: A dictionary specifying the initial position and orientation data.
    """
    global initial_positions
    global robots

    try:
        if not client.is_connected:
            client.run()
            
        action_client = roslibpy.actionlib.ActionClient(client, f"/launch_robot", 'limo_gazebo_sim/LaunchRobotAction')
            
        goal = roslibpy.actionlib.Goal(action_client, {
                'ns': int(robot_id), 
                'x': position['x'], 
                'y': position['y'], 
                'z': position['z'], 
                'yaw': position['yaw']
        })           
        goal.send()
        goal.wait(10)
        robots.add(robot_id)
        initial_positions[f"robot{robot_id}"] = position
        
        if len(robots) == 2:
            socket_service.socketio.emit('allSimConnected', True)

    except Exception as e:
        print(f"An error occurred in set_initial_position function: {str(e)}")
    

def return_to_base(robot=None):
    """
    Sends robots back to their base positions in the simulation environment.

    This function facilitates returning robots to their base positions within the simulation environment.
    If no specific robot is provided, it sends both 'robot1' and 'robot2' to their respective base positions.

    :param robot: A dictionary containing information about the robot.
    """
    try:
        global initial_positions

        if robot is None:
            for i in [1, 2]:
                send_return_to_base_goal(f"robot{i}", initial_positions[f'robot{i}'])
        else:
            robot_namespace = robot['name'].lower().replace(' ', '')
            if robot_namespace == 'robot1':
                send_return_to_base_goal(robot_namespace, initial_positions[robot_namespace])
            else:
                send_return_to_base_goal(robot_namespace, initial_positions[robot_namespace])
    except Exception as e:
        print(f"An error occurred in return_to_base function: {str(e)}")


def distance_callback(name, message, is_physical=False): 
    """
    Callback function invoked when a distance message is received from ROS.

    Handles incoming distance updates for robots and emits the information to the appropriate socket room.
    In the physical environment, it tracks distances of two robots and emits the total distance covered in the 'physical' room.
    In the simulation environment, it tracks distances of two simulated robots and emits the total distance covered in the 'simulation' room.

    :param name: The name or identifier of the robot.
    :param message: The message received from ROS with distance information.
    :param is_physical: Boolean indicating whether the environment is physical or simulated.
    """
    global are_two_robot_connected
    global dict_of_sim_robots
    global dict_of_physical_robots 

    try:
        if is_physical:
            if robot_controls.are_two_physical_launched:
                dict_of_physical_robots[name] = round(message['data'], 2)
                if len(dict_of_physical_robots) == 2:
                    socket_service.socketio.emit("receiveDistanceSim", f"Le robot 1 a parcouru {dict_of_physical_robots['robot1']} m et le robot 2 a parcouru {dict_of_physical_robots['robot2']} m en exploration physique", room='physical')
                    robot_controls.are_two_physical_launched = False
                    dict_of_physical_robots = {}
            else:
                socket_service.socketio.emit("receiveDistanceSim", f"Le {name[:-1] + ' ' + name[-1]} a parcouru {round(message['data'], 2)} m en exploration physique", room="192.168.0.110" if name=="robot1" else "192.168.0.122" )
        else:
            if are_two_robot_connected:
                dict_of_sim_robots[name] = round(message['total_distance'], 2)

                if len(dict_of_sim_robots) == 2:
                    socket_service.socketio.emit("receiveDistanceSim", f"Le robot 1 a parcouru {dict_of_sim_robots['robot1']} m et le robot 2 a parcouru {dict_of_sim_robots['robot2']} m en simulation", room="simulation")
                    are_two_robot_connected = False
                    dict_of_sim_robots = {}
            else:
                socket_service.socketio.emit("receiveDistanceSim", f"Le {name[:-1] + ' ' + name[-1]} a parcouru {round(message['total_distance'],2)} m en simulation", room="192.168.0.110sim" if name=="robot1" else "192.168.0.122sim" )
    except Exception as e:
        print(f"An error occurred in distance_callback function: {str(e)}")


def terminate_mission_robot(robot=None, unsubscribe=False):
    """
    Function to terminate the mission of a robot and handle necessary actions like stopping and unsubscribing.

    This function utilizes a nested function, `get_distance_and_stop`, to either stop the robot's movement
    or unsubscribe from certain topics based on the provided parameters.

    :param robot: Information about the robot whose mission needs to be terminated.
    :param unsubscribe: Boolean flag indicating whether to unsubscribe from specific topics or not.
    """
    global initial_positions
    global connected_robots
    global action_client
    global second_action_client
           
    def get_distance_and_stop(robot_name):
        try: 
            if unsubscribe:
                distance_query_action_client = roslibpy.actionlib.ActionClient(client, f"{robot_name}/distance_query", 'limo_gazebo_sim/DistanceQueryAction')
                goal = roslibpy.actionlib.Goal(distance_query_action_client, roslibpy.Message({}))
                goal.on('result', lambda message: distance_callback(robot_name, message))
                goal.send()
                goal.wait(5)
                roslibpy.Topic(client, f"/{robot_name}/odom", 'nav_msgs/Odometry').unsubscribe()
            else:
                if robot_name == "robot1" or robot is None: send_goal_and_wait(action_client, robot_name, True)
                else: send_goal_and_wait(second_action_client, robot_name, True)
        except Exception as e:
            print(f"An error occurred in get_distance_and_stop function: {str(e)}")

    try:
        if robot is None:
            for i in [1, 2]:
                robot_name = f"robot{i}"
                get_distance_and_stop(robot_name)
                    
            if unsubscribe and map_topic.is_subscribed: 
                map_topic.unsubscribe()
        else:
            get_distance_and_stop(robot['name'].lower().replace(' ', ''))
            if unsubscribe and map_topic.is_subscribed: map_topic.unsubscribe()
            if unsubscribe and second_map_topic.is_subscribed: second_map_topic.unsubscribe()
            
    except Exception as e:
        print(f"An error occurred in terminate_mission_robot function: {str(e)}")
