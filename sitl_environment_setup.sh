#MICROROS
directory=microros_ws
gnome-terminal --tab --working-directory="$directory" -- bash -c "export ROS_DOMAIN_ID=0; export PYTHONOPTIMIZE=1; source install/local_setup.sh; ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 ROS_DOMAIN_ID=0"

#PX4
directory=PX4-Autopilot
gnome-terminal --tab --working-directory="$directory" -- bash -c "export ROS_DOMAIN_ID=0; export PYTHONOPTIMIZE=1; make px4_sitl gazebo-classic_iris"
