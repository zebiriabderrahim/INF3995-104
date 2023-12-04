#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class RecoveryAlgorithm:
    def __init__(self):
        self.last_cmd_vel = Twist()
        self.timeout_duration = rospy.Duration(5.0)
        self.zero_velocity_received_time = None
        self.is_twisting = False  # Flag to indicate if twisting is in progress

        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, data):
        if self.is_twisting:
            # Ignore new cmd_vel messages during twisting
            rospy.loginfo("Ignoring new cmd_vel message during twisting.")
            return

        # Process cmd_vel as usual when not twisting
        self.last_cmd_vel = data
        if self.is_zero_velocity(data):
            if self.zero_velocity_received_time is None:
                self.zero_velocity_received_time = rospy.Time.now()
        else:
            self.zero_velocity_received_time = None

    def is_zero_velocity(self, twist):
        return (
            twist.linear.x == 0 and twist.linear.y == 0 and twist.linear.z == 0 and
            twist.angular.x == 0 and twist.angular.y == 0 and twist.angular.z == 0
        )

    def twist_robot_right(self):
        self.is_twisting = True  # Set the twisting flag
        initial_cmd_vel = self.last_cmd_vel

        twist = Twist()
        twist.angular.z = 0.75

        rate = rospy.Rate(1)
        duration = 5  # Reduced to 5 seconds for demonstration purposes
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time).to_sec() < duration:
            if initial_cmd_vel != self.last_cmd_vel:
                # If a new cmd_vel message is received during twisting, stop the twisting
                rospy.loginfo("Stopping twisting due to new cmd_vel message.")
                self.is_twisting = False
                return

            self.twist_pub.publish(twist)
            rate.sleep()

        self.is_twisting = False  # Reset the twisting flag

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.zero_velocity_received_time is not None:
                if (rospy.Time.now() - self.zero_velocity_received_time) > self.timeout_duration:
                    # Disable the cmd_vel callback while performing the twist
                    self.is_twisting = True
                    self.twist_robot_right()
                    self.zero_velocity_received_time = None
                    # Re-enable the cmd_vel callback
                    self.is_twisting = False
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('recovery_algo')
        recovery_algo = RecoveryAlgorithm()
        recovery_algo.run()
    except rospy.ROSInterruptException:
        pass

