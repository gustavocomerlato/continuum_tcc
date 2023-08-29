#!/bin/bash
if [ $# = 4 ]; then
	ros2 topic pub --once /secConfigParams std_msgs/msg/Float64MultiArray "{data: [$1, $2, $3, $4]}"
else
	str="{data: ["
	for inp in $*
	do
		str+="${inp}, "
	done
	str=${str::-2}
	str+="]}"
	#echo $str
	ros2 topic pub --once /secConfigParams std_msgs/msg/Float64MultiArray "$str"
fi
