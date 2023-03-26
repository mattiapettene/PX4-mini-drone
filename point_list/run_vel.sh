#!/bin/bash
colcon build
source install/local_setup.bash
ros2 run point_list_py vel_ctrl