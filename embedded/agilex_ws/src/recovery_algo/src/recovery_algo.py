#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray

class RecoveryAlgorithm:
    def __init__(self):
        self.last_cmd_vel = Twist()
        self.timeout_duration = rospy.Duration(5.0)
        self.zero_velocity_received_time = None
        self.is_cmd_vel_received = False

        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, data):
        self.is_cmd_vel_received = True
        self.last_cmd_vel = data
        if self.is_zero_velocity(data):
            if self.zero_velocity_received_time is None:
                self.zero_velocity_received_time = rospy.Time.now()
        else:
            self.zero_velocity_received_time = None

    def is_zero_velocity(self, twist):
        return twist.linear.x == 0 and twist.linear.y == 0 and twist.linear.z == 0 and twist.angular.x == 0 and twist.angular.y == 0 and twist.angular.z == 0

    def twist_robot_right(self):

        initial_cmd_vel = self.last_cmd_vel

        twist = Twist()
        twist.angular.z = 0.75

        rate = rospy.Rate(1)
        duration = 10
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time).to_sec() < duration:
            if not self.is_zero_velocity(self.last_cmd_vel):
                return
            if initial_cmd_vel != self.last_cmd_vel:
                return

            self.twist_pub.publish(twist)
            rate.sleep()


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.zero_velocity_received_time is not None:
                if (rospy.Time.now() - self.zero_velocity_received_time) > self.timeout_duration:
                    self.twist_robot_right()
                    self.zero_velocity_received_time = None
                    self.is_cmd_vel_received = False
            elif not self.is_cmd_vel_received:
                self.zero_velocity_received_time = None
            self.is_cmd_vel_received = False
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('recovery_algo')
        recovery_algo = RecoveryAlgorithm()
        recovery_algo.run()
    except rospy.ROSInterruptException:
        pass


