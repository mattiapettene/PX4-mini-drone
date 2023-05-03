# Hardware-in-the-loop (HITL) simulation
<br>
Hardware-in-the-loop (HITL) is a simulation mode in which normal PX4 firmware is run on a real flight controller hardware. The flight controller hardware is connected to Gazebo Classic via USB/UART.ù

## HITL setup

### QGroundControl configuration
First of all it is necessary to configure QGroundControl to enable the HITL mode:
- Connect the flight controller directly to QGroundControl via USB
- Enable HITL mode, openening <code> Setup > Safety </code>
- Select the airframe, choosing Setup > Airframes > HIL Quadcopter
- Calibrate the RC and enable it setting COM_RC_OVERRIDE = 3
Then close QGroundControl to proceed with the remaining setup.

### Companion board configuration
During the HITL simulation, the commands to the flight controller are sent by a companion board, which in our case is a Raspberry Pi 4. To configure it:
- Install ROS2 in the same way reported [here](https://github.com/mattiapettene/PX4-mini-drone/blob/main/docs/toolchain_installation.md)
- Clone the repo with the script to be launched

## HITL simulation
