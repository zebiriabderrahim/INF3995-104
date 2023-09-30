import time
import roslibpy

client = roslibpy.Ros(host='192.168.64.12', port=9090)

def identify_robot(robot):
    # TODO: To be implemented by robot team
    print("Identify command sent")
    return robot

def connect_same_topic(i,speed,cmd_topic):
    cmd_topic.advertise()
    linear_speed = speed
    msg = roslibpy.Message({
            'linear': {'x': linear_speed, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 1.0, 'y': 0.0, 'z': 0.0}
        })
    cmd_topic.publish(roslibpy.Message(msg))
    time.sleep(1)

def simulate_mission(speed):
    try:
        client.run()
        for i in range(0, 3):
            cmd_topic = roslibpy.Topic(client, '/robot' + str(i) + '/cmd_vel', 'geometry_msgs/Twist')
            connect_same_topic(i,speed,cmd_topic)
    except Exception as e:
        print(f"An error occurred: {str(e)}")


def terminate_mission(speed):
    try:
        for i in range(0, 3):
            cmd_topic = roslibpy.Topic(client, '/robot' + str(i) + '/cmd_vel', 'geometry_msgs/Twist')
            connect_same_topic(i,speed,cmd_topic)
            cmd_topic.unadvertise()
        client.terminate()
    except Exception as e:
        print(f"An error occurred: {str(e)}")