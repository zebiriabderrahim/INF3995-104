#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <limo_gazebo_sim/StopResumeExplorationAction.h>
#include <limo_gazebo_sim/LaunchRobotAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/process.hpp>
#include <string>

namespace bp = boost::process;

typedef actionlib::SimpleActionServer<limo_gazebo_sim::StopResumeExplorationAction> StopResumeActionServer;
typedef actionlib::SimpleActionServer<limo_gazebo_sim::LaunchRobotAction> LaunchRobotActionServer;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SimulationManagerServer
{
private:
    ros::NodeHandle nh_;
    StopResumeActionServer stop_resume_server_;
    LaunchRobotActionServer launch_robot_server_;

    void executeStopResumeCB(const limo_gazebo_sim::StopResumeExplorationGoalConstPtr &goal)
    {
        limo_gazebo_sim::StopResumeExplorationResult result;
        MoveBaseClient mb_client(goal->robot_namespace + "/move_base", true);

        if (goal->stop)
        {
            ROS_INFO("Stopping exploration for robot namespace: %s", goal->robot_namespace.c_str());
            std::string kill_cmd = "rosnode kill " + goal->robot_namespace + "/explore";
            if (system(kill_cmd.c_str()) != 0)
            {
                ROS_ERROR("Failed to stop exploration for robot namespace: %s", goal->robot_namespace.c_str());
                result.success = false;
                result.message = "Failed to stop exploration.";
            }
            else
            {
                mb_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
                ROS_INFO("Exploration successfully stopped for robot namespace: %s", goal->robot_namespace.c_str());
                result.success = true;
                result.message = "Exploration stopped for namespace: " + goal->robot_namespace;
            }
        }
        else
        {
            ROS_INFO("Resuming exploration for robot namespace: %s", goal->robot_namespace.c_str());
            std::string launch_cmd = "roslaunch limo_gazebo_sim explore_robot.launch ns:=" + goal->robot_namespace;
            try {
                bp::child c("/bin/bash", "-c", launch_cmd);
                c.detach(); // Detach the process to run independently

                result.success = true;
                result.message = "Exploration resumed for namespace: " + goal->robot_namespace;
                ROS_INFO("Exploration successfully resumed for robot namespace: %s", goal->robot_namespace.c_str());
            }
            catch(const std::exception& e) {
                ROS_ERROR("Failed to resume exploration for robot namespace: %s. Error: %s", goal->robot_namespace.c_str(), e.what());
                result.success = false;
                result.message = "Failed to resume exploration.";
            }
        }

        stop_resume_server_.setSucceeded(result);
    }

    void executeLaunchRobotCB(const limo_gazebo_sim::LaunchRobotGoalConstPtr &goal)
    {
        ROS_INFO("Launching robot for namespace: robot%d", goal->ns);
        limo_gazebo_sim::LaunchRobotResult result;
        
        std::string launch_cmd = "roslaunch limo_gazebo_sim simulation.launch ns:=robot" + std::to_string(goal->ns) +
                                 " x:=" + std::to_string(goal->x) + " y:=" + std::to_string(goal->y) +
                                 " z:=" + std::to_string(goal->z) + " yaw:=" + std::to_string(goal->yaw);

        ROS_INFO("cmd: %s", launch_cmd.c_str());
        try {
            bp::child c("/bin/bash", "-c", launch_cmd);
            c.detach(); 

            result.success = true;
            result.message = "Launch file executed for namespace: robot" + std::to_string(goal->ns);
            ROS_INFO("Robot successfully launched for namespace: robot%d", goal->ns);
        }
        catch(const std::exception& e) {
            ROS_ERROR("Failed to launch robot for namespace: robot%d. Error: %s", goal->ns, e.what());
            result.success = false;
            result.message = "Failed to launch robot.";
        }

        launch_robot_server_.setSucceeded(result);
    }

public:
    SimulationManagerServer()
        : stop_resume_server_(nh_, "stop_resume_exploration", boost::bind(&SimulationManagerServer::executeStopResumeCB, this, _1), false),
          launch_robot_server_(nh_, "launch_robot", boost::bind(&SimulationManagerServer::executeLaunchRobotCB, this, _1), false)
    {
        stop_resume_server_.start();
        launch_robot_server_.start();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulation_manager_server");
    ROS_INFO("Starting Simulation Manager Server");
    SimulationManagerServer server;
    ros::spin();
    return 0;
}