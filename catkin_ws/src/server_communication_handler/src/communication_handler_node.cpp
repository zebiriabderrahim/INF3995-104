#include "ros/ros.h"
#include "server_communication_handler/launch_identify_srv.h"
#include <sstream>

bool callback(server_communication_handler::launch_identify_srv::Request &req,
              server_communication_handler::launch_identify_srv::Response &res)
{
    std::stringstream ss;
    ss << "roslaunch " << req.in;
    std::system(ss.str().c_str());
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "communication_handler");
    // if (argc != 2)
    // {
    //     ROS_INFO("Usage: launcher .launch file ");
    //     return 1;
    // }
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("/launcher", callback);

    ROS_INFO("launcher_server has started");

    ros::spin();

    return 0;
}
