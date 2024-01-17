# Ultra wide band simulation

## [QGroundControl parameters setup](https://docs.px4.io/main/en/ros/external_position_estimation.html#using-vision-or-motion-capture-systems-for-position-estimation)

The following parameters must be set to use external position information with EKF2 (these can be set in QGroundControl > Vehicle Setup > Parameters > EKF2).

|**Parameters** | **Setting for External Position Estimation** |
| --- | --- |
| `UXRCE_DDS_CFG`| a desired port on your flight controller|
| `ser_tel1_baud`| `921600`|
| `EKF2_EV_CTRL` | Select *horizontal position fusion*, *vertical vision fusion* |
| `EKF2_HGT_REF` | Set to *Vision* to use the vision as the reference source for altitude estimation. |
| `EKF2_EV_POS_X,Y,Z` | Set the position of the vision sensor (or UWB tag) with respect to the robot's body frame. |
| `EKF2_GPS_CTRL`| Disable GPS sensor fusion|
| `EKF2_BARO_CTRL`| Disable Barometer sensor fusion|
| `EKF2_RNG_CTRL`| Disable range finder sensor fusion|

## Parameters Load (Instead of modifying it manually in QGroundControl)

To perform the simulation with UWB (external positioning system) you need first to setup the PX4 parameters in QGroundControl. You can load the parameters for the UWB simulation by opening QGroundControl and loading the parameters present in the folder **QGround_parameters**. In particular, you can load:

* [SITL_indoor_simu.params](/QGround_parameters/SITL_indoor_simu.params) - to perform Software in the loop simulations
* [HITL_indoor_simu.params](/QGround_parameters/HITL_intdoor_simu.params) - to perform Hardware in the loop simulations
* [indoor_test.params](/QGround_parameters/indoor_test.params) - to perform experimental simulations

## Simulations

Depending of the type of simulation that you want to perform, you need to setup the enviromnet following in different ways:

* [Software in the loop](/docs/sitl.md), skip the step where specified *(Only for experimental simulations)*
* [Hardware in the loop](/docs/hitl.md), skip the step where specified *(Only for experimental simulations)*
* Experimental tests: make sure that your drone is in Ready to fly state before lauch the simulation (see next steps)

To perfrom SITL and HITL simulations install first the **UWB plugin**, see section [plugin](/docs/plugins.md)

## Bridge UWB to PX4 (Only for SITL and HITL simulations)

On the command prompt enter

```bash
cd ~/PX4-mini-drone
ros2 run bridge_uwb_px4 uwb_bridge_sitl
```

This will start the bridge in order to fill the message fmu/in/vehicle_visual_odometry which is necessary for the external pose estimation

## Bridge UWB to PX4 (Only for experimental simulations)

**Before lauch the bridge make sure that your external postion reference frame system match the [drone reference frame](https://docs.px4.io/main/en/ros/external_position_estimation.html#reference-frames-and-ros), open [uwb_bridge](/bridge_uwb_px4/bridge_uwb_px4/uwb_bridge.py) [mb1202_bridge](/bridge_mb1202_px4/bridge_mb1202_px4/mb_1202_bridge.py) and modify if necessary**

On the command prompt enter

```bash
cd ~/PX4-mini-drone
ros2 launch launch/uwb_offboard.launch.py
```

This will start the uwb and mb1202 bridges in order to fill the message fmu/in/vehicle_visual_odometry which is necessary for the external pose estimation

## Run the program

You can run the same program used for the motion capture simulations

*Point target*
  
```bash
cd ~/PX4-mini-drone
ros2 run mcoap mocap
```

*Velocity target*
  
```bash
cd ~/PX4-mini-drone
ros2 run mcoap mocap_vel
```

*Square trajectory*

```bash
cd ~/PX4-mini-drone
ros2 run mcoap mocap_square
```