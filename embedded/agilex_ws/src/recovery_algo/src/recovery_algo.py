#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class RecoveryAlgorithm:
    def __init__(self):
        self.last_cmd_vel = Twist()
        self.timeout_duration = rospy.Duration(5.0)
        self.zero_velocity_received_time = None
        self.is_cmd_vel_received = False
        self.is_twisting = False

        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, data):
        if self.is_twisting:
            return

        self.is_cmd_vel_received = True
        self.last_cmd_vel = data

        if self.is_zero_velocity(data):
            if self.zero_velocity_received_time is None:
                self.zero_velocity_received_time = rospy.Time.now()
        else:
            if self.is_twisting_in_place(data):
                self.is_twisting = False
                return

            self.zero_velocity_received_time = None

    def twist_robot_right(self):
        self.is_twisting = True
        initial_cmd_vel = self.last_cmd_vel

        twist = Twist()
        twist.angular.z = 0.15

        rate = rospy.Rate(1)
        duration = 50
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time).to_sec() < duration:
            if self.is_significant_movement(self.last_cmd_vel) or self.is_non_twisting_movement(self.last_cmd_vel):
                self.is_twisting = False
                return

            if initial_cmd_vel != self.last_cmd_vel:
                self.is_twisting = False
                return

            if self.is_twisting:
                self.twist_pub.publish(twist)

            rate.sleep()

        self.is_twisting = False

    def is_zero_velocity(self, twist):
        return (
            twist.linear.x == 0 and
            twist.linear.y == 0 and
            twist.linear.z == 0 and
            twist.angular.x == 0 and
            twist.angular.y == 0 and
            twist.angular.z == 0
        )

    def is_significant_movement(self, twist):
        return twist.linear.x != 0 or twist.linear.y != 0 or (twist.angular.z != 0 and self.is_twisting_in_place(twist))

    def is_twisting_in_place(self, twist):
        return twist.linear.x == 0 and twist.linear.y == 0 and twist.linear.z == 0 and twist.angular.x == 0 and twist.angular.y == 0

    def is_non_twisting_movement(self, twist):
        return twist.linear.x != 0 or twist.linear.y != 0 or twist.linear.z != 0 or twist.angular.x != 0 or twist.angular.y != 0

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.zero_velocity_received_time is not None and not self.is_twisting:
                if (rospy.Time.now() - self.zero_velocity_received_time) > self.timeout_duration:
                    self.twist_robot_right()
                    self.zero_velocity_received_time = None
                    self.is_cmd_vel_received = False
            elif not self.is_cmd_vel_received and not self.is_twisting:
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

