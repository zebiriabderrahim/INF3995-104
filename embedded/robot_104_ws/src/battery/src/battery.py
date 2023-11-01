    #!/usr/bin/env python3
    # coding=UTF-8
    import rospy
    from std_msgs.msg import Float32

    from pylimo import limo as limo_module  # Rename the module to avoid conflict
    import time

    def publish_battery_percentage():
        # Initialize the ROS node
        rospy.init_node('battery_publisher', anonymous=True)
        
        # Create a publisher for the battery percentage
        battery_publisher = rospy.Publisher('/battery_percentage', Float32, queue_size=10)
        
        # Initialize LIMO
        limo = limo_module.LIMO()  # Rename the module here
        limo.EnableCommand()
        
        rate = rospy.Rate(1)  # 1 Hz
        
        while not rospy.is_shutdown():
            # Read the battery voltage
            voltage = limo.GetBatteryVoltage()
            rospy.loginfo(voltage)
            # Convert voltage to percentage (adjust these values as needed)
            max_voltage = 12.3
            min_voltage = 8.25
            battery_percentage = ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100.0
            
            # Publish the battery percentage
            battery_publisher.publish(battery_percentage)
            
            # Sleep to control the publishing rate
            rate.sleep()

    if __name__ == '__main__':
        try:
            publish_battery_percentage()
        except rospy.ROSInterruptException:
            pass