import time
import roslibpy
import roslibpy.actionlib
from services import socket_service, ros_utilities, socket_manager

client = roslibpy.Ros(host='192.168.64.12', port=9090)
last_processed_time = 0
launch_topic= ros_utilities.create_topic(client, 'launch_command')
all_robots_connected=False

def position_callback(robot_id, message, room_id):
    global last_processed_time
    current_time = time.time()

    if current_time - last_processed_time >= 1:  
        socket_service.socketio.emit("recieveSimRobotPos",{"robotId": str(robot_id), "position": message['pose']['pose']['position']}, room=room_id)
        print(message['pose']['pose']['position'])
        last_processed_time = current_time  

def simulate_robot_mission(robot=None):
    global all_robots_connected
    try:
        if not client.is_connected:
            client.run()

        if robot is None:
            all_robots_connected=True
            pos_topic_2 = roslibpy.Topic(client, '/robot2/odom', 'nav_msgs/Odometry')
            pos_topic_1 = roslibpy.Topic(client, '/robot1/odom', 'nav_msgs/Odometry')
            launch_topic.publish({'data': 'roslaunch limo_gazebo_sim explore_robot2.launch'})
            launch_topic.publish({'data': 'roslaunch limo_gazebo_sim explore_robot1.launch'})
            print(all_robots_connected)
        else:
            if robot["ipAddress"] == '192.168.0.110': 
                pos_topic = roslibpy.Topic(client, '/robot1/odom', 'nav_msgs/Odometry')
                launch_topic.publish({'data': 'roslaunch limo_gazebo_sim explore_robot1.launch'})
            else:
                pos_topic = roslibpy.Topic(client, '/robot2/odom', 'nav_msgs/Odometry')
                launch_topic.publish({'data': 'roslaunch limo_gazebo_sim explore_robot2.launch'})


        if robot is None:
            pos_topic_2.subscribe(lambda message: position_callback("2", message, 'simulation'))
            pos_topic_1.subscribe(lambda message: position_callback("1", message, 'simulation'))
        else:
            room_name = str(robot["ipAddress"] + 'sim')
            pos_topic.subscribe(lambda message: position_callback("1", message, room_name))

    except Exception as e:
        print(f"An error occurred: {str(e)}")
        if robot is None: 
            room = 'simulation'
            socket_service.socketio.emit('rosConnectionError', room=room)
            socket_manager.stop_simulation(robot, 'simulation')
        else: 
            room = str(robot["ipAddress"] + 'sim')
            socket_service.socketio.emit('rosConnectionError', room=room)
            socket_manager.stop_simulation(robot)


def terminate_mission_robot(robot=None):
    global all_robots_connected
    try:
        if all_robots_connected== True:
            launch_topic.publish({'data': 'rosnode kill /robot2/explore'})
            launch_topic.publish({'data': 'rosnode kill /robot1/explore'})
            all_robots_connected=False
            roslibpy.Topic(client, '/robot1/odom', 'nav_msgs/Odometry').unadvertise()
            roslibpy.Topic(client, '/robot2/odom', 'nav_msgs/Odometry').unadvertise()
        else:
            if robot["ipAddress"] == '192.168.0.110' :
                pos_topic = roslibpy.Topic(client, '/robot1/odom', 'nav_msgs/Odometry')
                launch_topic.publish({'data': 'rosnode kill /robot1/explore '})
            else : 
                pos_topic = roslibpy.Topic(client, '/robot2/odom', 'nav_msgs/Odometry')
                launch_topic.publish({'data': 'rosnode kill /robot2/explore'})
            pos_topic.unadvertise()
  
    except Exception as e:
        print(f"An error occurred: {str(e)}")