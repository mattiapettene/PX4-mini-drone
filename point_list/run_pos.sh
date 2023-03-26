#!/bin/bash
colcon build
source install/local_setup.bash
ros2 run point_list_py point_list_py