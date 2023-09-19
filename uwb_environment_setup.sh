#MICROXRCE
directory=Micro-XRCE-DDS-Agent
gnome-terminal --tab --working-directory="$directory" -- bash -c "MicroXRCEAgent udp4 -p 8888"

#PX4
directory=PX4-Autopilot
gnome-terminal --tab --working-directory="$directory" -- bash -c "make px4_sitl gazebo PX4_SITL_WORLD=my_world"
