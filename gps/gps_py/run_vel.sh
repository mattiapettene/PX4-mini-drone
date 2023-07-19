#!/bin/bash

source ~/px4_ros_com_ws/install/local_setup.bash
source install/local_setup.bash

ros2 run gps_py gps_velocity_py
