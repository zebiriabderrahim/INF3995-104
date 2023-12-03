#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def spin_robot():
    rospy.init_node('spin_robot')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    for count in range(8):  # CHANGE THIS VALUE TO TEST
        spin = Twist()
        spin.angular.z = 1  

        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(3.3):
            pub.publish(spin)
            rate.sleep()

        spin.angular.z = 0
        pub.publish(spin)
        rospy.sleep(1)  

if __name__ == '__main__':
    try:
        spin_robot()
    except rospy.ROSInterruptException:
        pass




