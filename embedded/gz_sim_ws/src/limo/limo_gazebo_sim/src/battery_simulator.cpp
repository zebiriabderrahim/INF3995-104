#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

float battery_percentage = 100.0;
float movement_drain_rate = 0.0;
float scan_drain_rate = 0.0;

// Callback for movement data
void movementCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    float speed = sqrt(pow(msg->linear.x, 2) + pow(msg->linear.y, 2) + pow(msg->linear.z, 2));
    movement_drain_rate = speed * 0.07; // Adjust the multiplier as needed
}

// Callback for scan data
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    int num_points = msg->ranges.size();
    scan_drain_rate = num_points * 0.0005; // Adjust the multiplier as needed
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "battery_simulator");
    ros::NodeHandle nh;
    std::string ns = ros::this_node::getNamespace();
    ros::Publisher battery_pub = nh.advertise<std_msgs::Float32>(ns+"/battery_percentage", 10);
    ros::Subscriber movement_sub = nh.subscribe(ns+"/cmd_vel", 10, movementCallback);
    ros::Subscriber scan_sub = nh.subscribe(ns+"/scan", 10, scanCallback);

    ros::Rate loop_rate(1); // 1 Hz update rate

    while (ros::ok()) {
        std_msgs::Float32 msg;

        battery_percentage -= (movement_drain_rate + scan_drain_rate);
        if (battery_percentage < 0) battery_percentage = 0;

        msg.data = battery_percentage;
        battery_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
