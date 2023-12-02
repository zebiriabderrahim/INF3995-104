#!/bin/bash

rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- '{}' && sleep 1
rostopic pub -8 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]' && sleep 1
nodes_to_kill=(
  "/explore"
  "/recovery_algo"
  "/base_link_to_camera_link"
  "/base_link_to_imu_link"
  "/base_link_to_laser_link"
  "/limo_base_node"
  "/move_base"
  "/robot_pose_ekf"
  "/rviz"
  "/slam_gmapping"
  "/ydlidar_node"
  "/map_merge"
  "/map_republisher"
  "/odom_distance_calculator"
  "/rosbridge_map_republisher"
)

for node in "${nodes_to_kill[@]}"; do
  echo "Killing node: $node"
  rosnode kill $node
done

echo "Nodes killed."

