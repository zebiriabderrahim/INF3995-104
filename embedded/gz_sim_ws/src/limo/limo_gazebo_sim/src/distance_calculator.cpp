#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <limo_gazebo_sim/DistanceQueryAction.h> // Replace with your actual package name
#include <cmath>
#include <string>

class OdomDistanceCalculator {
protected:
    ros::NodeHandle nh;
    std::string ns;
    ros::Subscriber odom_sub;
    double total_distance;
    double last_x, last_y, last_z;
    bool first_measurement;

    typedef actionlib::SimpleActionServer<limo_gazebo_sim::DistanceQueryAction> Server; // Replace with your actual package name
    Server action_server;
    limo_gazebo_sim::DistanceQueryResult result; // Replace with your actual package name

public:
    OdomDistanceCalculator(std::string name) : 
        action_server(nh, name, boost::bind(&OdomDistanceCalculator::executeCallback, this, _1), false),
        total_distance(0.0), 
        last_x(0.0), 
        last_y(0.0), 
        last_z(0.0), 
        first_measurement(true) {

        ros::NodeHandle private_nh("~");
        ns = ros::this_node::getNamespace();
        ROS_INFO("Namespace parameter (ns): %s", ns.c_str());
        std::string odom_topic = ns.empty() ? "/odom" : ns + "/odom";
        odom_sub = nh.subscribe(odom_topic, 1000, &OdomDistanceCalculator::odomCallback, this);
        ROS_INFO("Subscribed to odometry topic: %s", odom_topic.c_str());
        action_server.start();
        ROS_INFO("Action server started for distance queries.");
    }

    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;

        ROS_DEBUG("Received Odometry: x = %f, y = %f, z = %f", x, y, z);

        if (first_measurement) {
            ROS_DEBUG("Initializing first measurement.");
            last_x = x;
            last_y = y;
            last_z = z;
            first_measurement = false;
        } else {
            double distance_increment = sqrt(pow(x - last_x, 2) + pow(y - last_y, 2) + pow(z - last_z, 2));
            total_distance += distance_increment;
            ROS_DEBUG("Distance increment: %f meters, Total distance: %f meters", distance_increment, total_distance);
            last_x = x;
            last_y = y;
            last_z = z;
        }
    }

    void executeCallback(const limo_gazebo_sim::DistanceQueryGoalConstPtr &goal) {
        ROS_INFO("Received query for total distance.");
        result.total_distance = total_distance;
        action_server.setSucceeded(result);
        ROS_INFO("Responded with total distance: %f meters", total_distance);

        // Reset the total distance after providing the result
        ROS_INFO("Resetting total distance counter.");
        total_distance = 0.0;
        first_measurement = true;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_distance_calculator");

    OdomDistanceCalculator calculator("distance_query");

    ros::spin();

    return 0;
}
