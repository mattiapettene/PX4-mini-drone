# Software in the loop

To run the software in the loop simulation you need to open three different terminal.

1) Run the gazebo environment and start the Micro-XRCE-DDS-Agent:

Run the script

```bash
cd ~/PX4-mini-drone
chmod a+x sitl_enviroment_setup.sh
./sitl_enviroment_setup.sh
```

or alternatively

```bash
cd ~/PX4-mini-drone
make px4_sitl gazebo
```

```bash
cd ~/PX4-mini-drone
MicroXRCEAgent udp4 -p 8888
```

2) Then the package has to be runned:

```bash
source ~/ws_combined_sensor/install/local_setup.bash
source install/local_setup.bash

ros2 run <package_name> <executable_name>
```
