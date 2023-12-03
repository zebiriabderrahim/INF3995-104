import time
import roslibpy
import roslibpy.actionlib
from services import socket_service, socket_manager, robot_controls
from database import database_communication

client = roslibpy.Ros(host='192.168.80.2', port=9090)
# client = roslibpy.Ros(host='ros_gazebo_simulation_container', port=9090)
map_topic = roslibpy.Topic(client, '/map', 'nav_msgs/OccupancyGrid')
second_map_topic = roslibpy.Topic(client, '/map', 'nav_msgs/OccupancyGrid')
last_processed_time = 0
last_processed_battery_time = 0
initial_positions = {}
return_base_robots = set()
robots = set()
current_positions = [[0.0,0.0],[0.0,0.0]]
battery_on = {"192.168.0.110": "off", "192.168.0.122": "off"}
are_two_robot_connected = False
dict_of_sim_robots = {}
dict_of_physical_robots = {}



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
    global battery_on
    try:
        if(battery_on[robot["ipAddress"]] == 'off'):
            roslibpy.Topic(client, f"/{robot['name'].lower().replace(' ', '')}/battery_percentage", 'std_msgs/Float32').subscribe(lambda message: battery_callback(robot, message))
            print("i m here")
            battery_on[robot["ipAddress"]] = 'on'
    except Exception as e:
        print(f"An error occurred in subscribe_to_battery function: {str(e)}")


def battery_callback(robot, message):
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
    global last_processed_time, current_positions
    current_time = time.time()

    if current_time - last_processed_time >= 1:  
        current_positions[int(robot_id)-1][0] = message["pose"]["pose"]["position"]["x"]
        current_positions[int(robot_id)-1][1] = message["pose"]["pose"]["position"]["y"]
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
    try:            
        action_client = roslibpy.actionlib.ActionClient(client, '/stop_resume_exploration', 'limo_gazebo_sim/StopResumeExplorationAction')
        if robot is None:
            are_two_robot_connected = True
            return_base_robots.clear()
            roslibpy.Topic(client, '/robot2/odom', 'nav_msgs/Odometry').subscribe(lambda message: position_callback("2", message, 'simulation'))
            roslibpy.Topic(client, '/robot1/odom', 'nav_msgs/Odometry').subscribe(lambda message: position_callback("1", message, 'simulation'))
            for i in [1, 2]:
                send_goal_and_wait(action_client, f"robot{i}", False)
        
        else:
            roslibpy.Topic(client, f"{robot['name'].lower().replace(' ', '')}/odom", 'nav_msgs/Odometry').subscribe(lambda message: position_callback(robot["name"][-1], message, str(robot["ipAddress"] + 'sim')))
            send_goal_and_wait(action_client, f"{robot['name'].lower().replace(' ', '')}", False)      
            if return_base_robots.__contains__(robot['name'].lower().replace(' ', '')): return_base_robots.remove(robot['name'].lower().replace(' ', ''))
            
        action_client.dispose()
        if robot is None: 
            map_topic.subscribe(lambda message: socket_manager.map_callback(message, 'simulation'))
        else:           
            if robot['name']== "Robot 1": 
                 map_topic.subscribe(lambda message: socket_manager.map_callback(message, str(robot["ipAddress"] + 'sim')))
            else:
                 second_map_topic.subscribe(lambda message: socket_manager.map_callback(message, str(robot["ipAddress"] + 'sim')))
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


def distance_callback(name, message,is_physical=False): 
    global are_two_robot_connected
    global dict_of_sim_robots
    global dict_of_physical_robots
    global physical_robot_distance 
    global simulated_robot_distance

    try:
        if is_physical:
            room_name = ''
            print("physical value",physical_robot_distance)         
            if robot_controls.are_two_physical_launched:
                dict_of_physical_robots[name] = round(message['data'], 2)
                print(dict_of_physical_robots)
                if len(dict_of_physical_robots) == 2:
                    room_name = 'physical'
                    physical_robot_distance=f"Le robot 1 a parcouru {dict_of_physical_robots['Robot 1']} m et le robot 2 a parcouru {dict_of_physical_robots['Robot 1']} m en exploration physique"
                    robot_controls.are_two_physical_launched = False
                    dict_of_physical_robots = {}
            else:
                if name == "Robot 1":
                    room_name = "192.168.0.110"
                else: 
                    room_name = "192.168.0.122"
                physical_robot_distance=f"Le {name[:-1].lower() + ' ' + name[-1]} a parcouru {round(message['data'], 2)} m en exploration physique"
            socket_service.socketio.emit("receiveDistanceSim", physical_robot_distance, room=room_name) 
            socket_service.socketio.close_room(room_name) 
        else:
            if are_two_robot_connected:
                dict_of_sim_robots[name] = round(message['total_distance'], 2)

                if len(dict_of_sim_robots) == 2:
                    simulated_robot_distance=f"Le robot 1 a parcouru {dict_of_sim_robots['robot1']} m et le robot 2 a parcouru {dict_of_sim_robots['robot2']} m en simulation"
                    are_two_robot_connected = False
                    dict_of_sim_robots = {}
            else:
                simulated_robot_distance=f"Le {name[:-1] + ' ' + name[-1]} a parcouru {round(message['total_distance'],2)} m en simulation"
    except Exception as e:
        print(f"An error occurred in distance_callback function: {str(e)}")


def terminate_mission_robot(robot=None):
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
        distance_query_action_client = roslibpy.actionlib.ActionClient(client, f"{robot_name}/distance_query",'limo_gazebo_sim/DistanceQueryAction')
        goal = roslibpy.actionlib.Goal(distance_query_action_client, roslibpy.Message({}))
        goal.on('result', lambda message: distance_callback(robot_name, message))
        goal.send()

        roslibpy.Topic(client, f"/{robot_name}/odom", 'nav_msgs/Odometry').unsubscribe()
        send_goal_and_wait(stop_resume_action_client, robot_name, True)
        distance_query_action_client.dispose()

    try:
        if robot is None:
            for i in [1, 2]:
                robot_name = f"robot{i}"
                get_distance_and_stop(robot_name)
        else:
            print("in get distance and stop for", robot['name'].lower().replace(' ', ''))   
            get_distance_and_stop(robot['name'].lower().replace(' ', ''))
            if unsubscribe and map_topic.is_subscribed: map_topic.unsubscribe()
            if unsubscribe and second_map_topic.is_subscribed: second_map_topic.unsubscribe()
            
        if robots is None or robot['name']=="Robot 1": map_topic.unsubscribe()
        stop_resume_action_client.dispose()
    except Exception as e:
        print(f"An error occurred in terminate_mission_robot function: {str(e)}")
