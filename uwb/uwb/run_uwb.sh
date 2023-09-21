#!/bin/bash

colcon build
source install/local_setup.bash
ros2 run uwb uwb
