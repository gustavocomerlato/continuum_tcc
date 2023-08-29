#!/bin/bash
ros2 topic pub --once /configParams std_msgs/msg/Float64MultiArray "{data: [$1, $2, $3, $4]}"
