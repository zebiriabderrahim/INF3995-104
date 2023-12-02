#!/usr/bin/env python3
# coding=UTF-8
import rospy
from std_msgs.msg import Float32
from pylimo import limo as limo_module
import time

class BatteryMonitor:
    def __init__(self):
        self.battery_publisher = rospy.Publisher('battery_percentage', Float32, queue_size=10)
        
        self.initialize_limo()

        self.zero_voltage_count = 0
        self.zero_voltage_threshold = 3

    def initialize_limo(self):
        try:
            self.limo = limo_module.LIMO()

            self.limo.EnableCommand()

            rospy.loginfo("LIMO module initialized successfully.")
        except Exception as e:
            rospy.logerr("Error initializing LIMO module:", str(e))
            rospy.signal_shutdown("Error initializing LIMO module. Exiting.")
            raise

    def restart_limo_module(self):
        rospy.loginfo("Restarting LIMO module...")
        try:
            self.initialize_limo()
            self.zero_voltage_count = 0
            rospy.loginfo("LIMO module restarted successfully.")
        except Exception as e:
            rospy.logerr("Error restarting LIMO module:", str(e))
            rospy.logerr("Exiting due to LIMO module restart failure.")
            rospy.signal_shutdown("LIMO module restart failure. Exiting.")
            raise

    def publish_battery_percentage(self):
        rate = rospy.Rate(1)  # 1 Hz

        while not rospy.is_shutdown():
            voltage = self.limo.GetBatteryVoltage()
            rospy.loginfo(voltage)

            if voltage == 0:
                rospy.logwarn("Received 0 voltage. Retrying...")
                self.zero_voltage_count += 1
                if self.zero_voltage_count >= self.zero_voltage_threshold:
                    self.restart_limo_module()
                continue
            else:
                self.zero_voltage_count = 0

            max_voltage = 12.3
            min_voltage = 8.25
            battery_percentage = ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100.0

            self.battery_publisher.publish(battery_percentage)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('battery_publisher')
        battery_monitor = BatteryMonitor()
        battery_monitor.publish_battery_percentage()
    except rospy.ROSInterruptException:
        pass

