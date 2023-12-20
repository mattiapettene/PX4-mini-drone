# Motion capture simulation (Qualysis)

## [QGroundControl parameters setup](https://docs.px4.io/main/en/ros/external_position_estimation.html#using-vision-or-motion-capture-systems-for-position-estimation)

The following parameters must be set to use external position information with EKF2 (these can be set in QGroundControl > Vehicle Setup > Parameters > EKF2).

|**Parameters** | **Setting for External Position Estimation** |
| --- | --- |
| `UXRCE_DDS_CFG`| a desired port on your flight controller|
| `ser_tel1_baud`| `921600`|
| `EKF2_EV_CTRL` | Select *horizontal position fusion*, *vertical vision fusion* |
| `EKF2_HGT_REF` | Set to *Vision* to use the vision as the reference source for altitude estimation. |
| `EKF2_EV_POS_X,Y,Z` | Set the position of the vision sensor (or MoCap markers) with respect to the robot's body frame. |
| `EKF2_GPS_CTRL`| Disable GPS sensor fusion|
| `EKF2_BARO_CTRL`| Disable Barometer sensor fusion|
| `EKF2_RNG_CTRL`| Disable range finder sensor fusion|

## Select the qualysis topic

In order to publish the px4 message with the external positon estimate you have to select the correct topic where qualysis publish is message.

```bash
cd ~/PX4-mini-drone/bridge_mocap_px4/src/
nano mocap_bridge.cpp
```

Modify the line ```	std::string topic_name = "/Drone/pose"``` with your topic name.

## Bridge motion capture to PX4

On the command prompt enter

```bash
cd ~/PX4-mini-drone
ros2 launch launch/mocap_offboard.launch.py
```

This will start the qualysis plugin and start the bridge in order to fill the message fmu/in/vehicle_visual_odometry which is necessary for the external pose estimation

## Run the program

Start the simulation, you can choose two different control strategy:

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
