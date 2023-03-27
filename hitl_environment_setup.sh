#PX4
directory=PX4-Autopilot
gnome-terminal --tab --working-directory="$directory" -- bash -c "DONT_RUN=1 make px4_sitl_default gazebo-classic; source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default; gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world "


