#MICROROS
directory=microros_ws
gnome-terminal --tab --working-directory="$directory" -- bash -c "MicroXRCEAgent udp4 -p 8888"

#PX4
directory=PX4-Autopilot
gnome-terminal --tab --working-directory="$directory" -- bash -c "make px4_sitl gz_x500 PX4_SITL_WORLD=my_world"
