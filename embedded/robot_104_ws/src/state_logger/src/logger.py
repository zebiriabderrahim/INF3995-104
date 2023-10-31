#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from statelogger.msg import LogMessage
from time import strftime

def log_callback(data):
    # Create a LogMessage
    log_msg = LogMessage()
    log_msg.timestamp = rospy.Time.now()
    log_msg.message = f"[{strftime('%Y-%m-%d %H:%M:%S')}] {data.data}"

    # Publish the LogMessage to the logging topic
    log_pub.publish(log_msg)

if __name__ == '__main__':
    rospy.init_node('state_logger')
    log_pub = rospy.Publisher('/log_topic', LogMessage, queue_size=10)
    rospy.Subscriber('/incoming_topic', String, log_callback)
    rospy.spin()

