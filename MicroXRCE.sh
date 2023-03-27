#!/bin/bash

source ~/microros_ws/install/local_setup.bash
source install/local_setup.bash

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyS0 -b 921600 ROS_DOMAIN_ID=0
