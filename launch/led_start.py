#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def control_led():
    rospy.init_node('led_controller', anonymous=True)
    pub = rospy.Publisher('/led_control', Bool, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    led_state = False

    while not rospy.is_shutdown():
        led_state = not led_state  # Toggle the LED state
        pub.publish(Bool(led_state))
        rate.sleep()

if __name__ == '__main__':
    try:
        control_led()
    except rospy.ROSInterruptException:
        pass
