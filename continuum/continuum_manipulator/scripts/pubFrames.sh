#!/bin/bash
ros2 topic pub --once /distalFrame geometry_msgs/msg/Transform "{translation: {x: $1, y: $2, z: $3}, rotation: {x: 0, y: 0, z: 0, w: 1}}"

