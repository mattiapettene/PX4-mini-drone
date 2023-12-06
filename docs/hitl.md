# Hardware-in-the-loop (HITL) simulation

Hardware-in-the-loop (HITL) is a simulation mode in which normal PX4 firmware is run on a real flight controller hardware. The flight controller hardware is connected to Gazebo Classic via USB/UART.

## HITL setup

### QGroundControl configuration

First of all it is necessary to configure QGroundControl to enable the HITL mode:

- Connect the flight controller directly to QGroundControl via USB
- Enable HITL mode, openening ```Setup > Safety```
- Select the airframe, choosing Setup > Airframes > HIL Quadcopter
- Calibrate the RC and enable it setting COM_RC_OVERRIDE = 3
- Make sure to disable all the MAVLink ports and set Micro XRCE to TELEM1, boudrate = 921600

Then close QGroundControl to proceed with the remaining setup.

### Companion board configuration

During the HITL simulation, the commands to the flight controller are sent by a companion board, which in our case is a Raspberry Pi 4. To configure it:

- Install ubuntu server [Guide](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview)
- Install ROS2 and micro-ROS (or MicroXRCE) in the same way reported [here](https://github.com/mattiapettene/PX4-mini-drone/blob/main/docs/toolchain_installation.md)
- Clone the [repository](https://github.com/mattiapettene/PX4-mini-drone/tree/main) with the script to be launched

## HITL simulation

### Gazebo launch

Make sure to do not have any gazebo client or server opened:

``` bash
killall gzclient
killall gzserver
```

Make sure QGroundControl is closed. Then launch the following from a terminal:

``` bash
cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo-classic 
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world
```

Only at this point it is possible to launch QGroundControl.

### ROS2 launch

On the companion board, launch ROS2:

``` bash
cd ~/PX4-mini-drone
sudo MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600 ROS_DOMAIN_ID=0
```

At this point, always on the companion board, it is possible to navigate to the folder where the scripts are contained and launch them in the same way as in the SITL simulation.
