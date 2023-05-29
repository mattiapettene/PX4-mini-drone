# Hardware-in-the-loop (HITL) simulation
<br>
Hardware-in-the-loop (HITL) is a simulation mode in which normal PX4 firmware is run on a real flight controller hardware. The flight controller hardware is connected to Gazebo Classic via USB/UART.

## HITL setup

### QGroundControl configuration
First of all it is necessary to configure QGroundControl to enable the HITL mode:
- Connect the flight controller directly to QGroundControl via USB
- Enable HITL mode, openening <code> Setup > Safety </code>
- Select the airframe, choosing Setup > Airframes > HIL Quadcopter
- Calibrate the RC and enable it setting COM_RC_OVERRIDE = 3
- Make sure to disable all the MAVLink ports and set Micro XRCE to TELEM1, boudrate = 921600

Then close QGroundControl to proceed with the remaining setup.

### Companion board configuration
During the HITL simulation, the commands to the flight controller are sent by a companion board, which in our case is a Raspberry Pi 4. To configure it:
- Install ROS2 in the same way reported [here](https://github.com/mattiapettene/PX4-mini-drone/blob/main/docs/toolchain_installation.md)
- Clone the repo with the script to be launched

## HITL simulation
### Gazebo launch

Make sure to do not have any gazebo client or server opened:
``` bash
killall gzclien
killall gzserver
``` 

First of all make sure QGroundControl is closed. Then launch the following from a terminal: <br>
``` 
cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo-classic 
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world
```

Only at this point it is possible to launch QGroundControl.

### ROS2 launch
On the companion board, launch ROS2:
```
cd ~/PX4-mini-drone
source ~/microros_ws/install/local_setup.bash
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyS0 -b 921600 ROS_DOMAIN_ID=0
```
Alternatively it is possible to launch the following:
``` bash
cd ~/PX4-mini-drone
./MicroXRCE.sh
``` 

At this point, always on the companion board, it is possible to navigate to the folder where the scripts are contained and launch them in the same way as in the SITL simulation.
