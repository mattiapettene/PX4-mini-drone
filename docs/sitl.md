# Software in the loop

To run the software in the loop simulation you need to open three different terminal.

1) To start the software in the loop simulation the enviroment need to be setup:

```bash
cd PX4-mini-drone
chmod a+x sitl_enviroment_setup.sh
./sitl_enviroment_setup.sh
```

2) Start the Micro-XRCE-DDS-Agent:

``` bash
cd ~/Micro-XRCE-DDS-Agent
MicroXRCEAgent udp4 -p 8888
```

3) Then the package has to be runned:

```bash
source ~/px4_ros_com_ws/install/local_setup.bash
source install/local_setup.bash

ros2 run <package_name> <executable_name>
```
