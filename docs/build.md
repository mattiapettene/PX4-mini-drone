# [Build the code](https://docs.px4.io/main/en/dev_setup/building_px4.html)

The full syntax to call make with a particular configuration and initialization file is:

``` make
make [VENDOR_][MODEL][_VARIANT] [VIEWER_MODEL_DEBUGGER_WORLD]
```

VENDOR_MODEL_VARIANT:

* **VENDOR**: The manufacturer of the board: *px4*, *aerotenna*, *airmind*, *atlflight*, *auav*, *beaglebone*, *intel*, *nxp*, etc. The vendor name for Pixhawk series boards is *px4*.
* **MODEL**: The board model "model": *sitl*, *fmu-v2*, *fmu-v3*, *fmu-v4*, *fmu-v5*, *navio2*, etc.
* **VARIANT**: Indicates particular configurations: e.g. *bootloader*, *cyphal*, which contain components that are not present in the default configuration. Most commonly this is *default*, and may be omitted.

VIEWER_MODEL_DEBUGGER_WORLD:

* **VIEWER**: This is the simulator ("viewer") to launch and connect: *gz*, *gazebo*, *jmavsim*, none
* **MODEL**: The vehicle model to use (e.g. *iris* (default), *rover*, *tailsitter*, etc), which will be loaded by the simulator.
* **DEBUGGER**: Debugger to use: none (default), *ide*, *gdb*, *lldb*, *ddd*, *valgrind*, *callgrind*.
* **WORLD**: (Gazebo Classic only).

## Build the code for software in the loop simulation (SITL)

To build the code and run the simulation in gazebo:

``` make
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

## Build the code for hardware in the loop simulation (HITL)

To build the code for a specific target:

``` make
cd ~/PX4-Autopilot
make px4_fmu-v5_default
```

Then upload the firmware to the autopilot, remember to plug in the autopilot

``` make
cd ~/PX4-Autopilot
make px4_fmu-v5_default upload
```
