#!/usr/bin/env python3
import subprocess
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf

def kill_ros_node(node_name):
    try:
        subprocess.check_call(["rosnode", "kill", node_name])
        rospy.loginfo(f"Node {node_name} has been killed.")
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"Failed to kill node {node_name}: {e}")

class RobotController(object):
    def __init__(self):
        rospy.init_node('robot_controller')
        rospy.loginfo('INIT')
        self.mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.mb_client.wait_for_server()

    def go_to(self, x, y, theta):
        rospy.loginfo('GOING TO GOAL') 
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        self.mb_client.send_goal(goal)
        wait = self.mb_client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.mb_client.get_result()

    def stop_robot(self):
        rospy.loginfo('Cancel any existing goals')
        self.mb_client.cancel_all_goals()


if __name__ == '__main__':
    controller = RobotController()
    try:
        rospy.loginfo('EXPLORE Node Killer Initialized')
        node_to_kill = "/explore"
        kill_ros_node(node_to_kill)
        rospy.loginfo('EXPLORE Node Killed')
        controller.stop_robot()
        rospy.sleep(2)
        result = controller.go_to(0.0, 0.0, 0.0)
        if result:
            rospy.loginfo("Goal execution done!")
            kill_ros_node("/recovery_algo")
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupted.")
