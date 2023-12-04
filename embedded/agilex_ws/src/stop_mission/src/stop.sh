#!/bin/bash

MOVE_BASE_CANCEL_TOPIC="/move_base/cancel"
CMD_VEL_TOPIC="/cmd_vel"
DURATION=30  # Duration in seconds

# Calculate the number of iterations based on sleep duration
ITERATIONS=$(echo "$DURATION / 0.2" | bc)

# Send stop commands continuously for 30 seconds
for ((i=0; i<$ITERATIONS; i++)); do
	    rostopic pub -1 $MOVE_BASE_CANCEL_TOPIC actionlib_msgs/GoalID -- '{}' && sleep 0.1
	        rostopic pub -1 $CMD_VEL_TOPIC geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
		    sleep 0.2  # Increased sleep time to ensure that the loop runs for the specified duration
	    done

