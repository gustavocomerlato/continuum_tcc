#!/bin/bash
ros2 topic pub --once /secTangledCables std_msgs/msg/Float64MultiArray "{data: [$1, $2, $3, $4, $5]}"
