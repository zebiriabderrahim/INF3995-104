#!/usr/bin/env python3

import json
import rospy
import roslibpy
from nav_msgs.msg import OccupancyGrid

def map_callback(message):
    rospy_msg = OccupancyGrid()
    rospy_msg.header.stamp = rospy.Time.now()
    rospy_msg.header.frame_id = message['header']['frame_id']

    rospy_msg.info.map_load_time = rospy.Time.now()
    rospy_msg.info.resolution = message['info']['resolution']
    rospy_msg.info.width = message['info']['width']
    rospy_msg.info.height = message['info']['height']
    rospy_msg.info.origin.position.x = message['info']['origin']['position']['x']
    rospy_msg.info.origin.position.y = message['info']['origin']['position']['y']
    rospy_msg.info.origin.position.z = message['info']['origin']['position']['z']
    rospy_msg.info.origin.orientation.x = message['info']['origin']['orientation']['x']
    rospy_msg.info.origin.orientation.y = message['info']['origin']['orientation']['y']
    rospy_msg.info.origin.orientation.z = message['info']['origin']['orientation']['z']
    rospy_msg.info.origin.orientation.w = message['info']['origin']['orientation']['w']

    rospy_msg.data = message['data']

    local_publisher.publish(rospy_msg)
    print("Republished /robot2/map message locally.")

def main():
    global local_publisher
    rospy.init_node('rosbridge_map_republisher')
    local_publisher = rospy.Publisher('robot2/map', OccupancyGrid, queue_size=10)

    ros = roslibpy.Ros(host='192.168.0.122', port=9091)
    ros.run()

    map_topic = roslibpy.Topic(ros, '/robot2/map', 'nav_msgs/OccupancyGrid')
    map_topic.subscribe(map_callback)

    print("Connected to ROSBridge. Waiting for /robot2/map messages...")
    rospy.spin()

if __name__ == '__main__':
    main()

