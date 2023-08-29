#!/bin/bash
ros2 topic pub --once /continuum_section_$1/joint_states sensor_msgs/msg/JointState "{position: [$2, $3, $3, $3, -$2], velocity: [$4, $5, $5, $5, -$4]}"

