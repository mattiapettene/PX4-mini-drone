#!/bin/bash

cd ~/microros_ws
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
source install/local_setup.sh
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 ROS_DOMAIN_ID=0