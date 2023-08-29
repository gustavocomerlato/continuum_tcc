#!/bin/bash
if [ $# -eq 3 ]
then
	ros2 topic pub --once /cableLengths std_msgs/msg/Float64MultiArray "{data: [$1, $2, $3]}"
else
	ros2 topic pub --once /cableLengths std_msgs/msg/Float64MultiArray "{data: [$1, $2, $3, $4]}"
fi
