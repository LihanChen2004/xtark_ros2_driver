#!/bin/bash

source /opt/ros/humble/install/setup.bash \
source install/setup.bash \
ros2 launch xtark_robot robot.launch.py


cmds=(
	"source /opt/ros/humble/install/setup.bash"
	"source install/setup.bash"
    "ros2 launch xtark_robot robot.launch.py"
)

for cmd in "${cmds[@]}"
do
    echo Current CMD : "$cmd"
    eval "$cmd"
    sleep 0.2
done