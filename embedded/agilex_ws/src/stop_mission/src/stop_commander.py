#!/bin/bash

declare -a PACKAGES=("tf" "robot_pose_ekf" "gmapping" "move_base" "explore_lite")

NODES=$(rosnode list)

for NODE in $NODES; do
  INFO=$(rosnode info $NODE)
  
  for PACKAGE in "${PACKAGES[@]}"; do
    if echo "$INFO" | grep -qw "$PACKAGE"; then
      rosnode kill $NODE
      break
    fi
  done
done
