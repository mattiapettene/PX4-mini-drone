# Plugins

## UWB Plugin (UWB SITL simulations)

To use the uwb gazebo plugin you need to install and build the following repositories:

- [rosmsgs](https://github.com/GiacomoCorradini/rosmsgs)
- [UWB gazebo plugin](https://github.com/GiacomoCorradini/uwb_gazebo_plugin)

Then to perform the simulation copy the customize world inside the PX4-Autopilot folder:

```bash
cp ~/PX4-mini-drone/world/my_world.world ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/
```

You can also create your custom world and copy it to the PX4 folder ```~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/```

To use the uwb plugin, add the following lines befor ```</model>``` to ```~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris.sdf``` file

```xml
<plugin name='libgtec_uwb_plugin' filename='libgtec_uwb_plugin.so'>
    <tag_link>base_link</tag_link>
    <update_rate>50</update_rate>
    <nlosSoftWallWidth>0.25</nlosSoftWallWidth>
    <tag_z_offset>0</tag_z_offset>
    <anchor_prefix>uwb_anchor</anchor_prefix>
    <all_los>true</all_los>
    <tag_id>0</tag_id>
</plugin>
```

To lunch the simulation including the UWB world:

```bash
make px4_sitl gazebo-classic_iris PX4_SITL_WORLD=my_world
```

## Position plugin (MoCap SITL simulations)

To use the position gazebo plugin you need to install and build the following repository:

- [position_plugin](https://github.com/GiacomoCorradini/position_gazebo_plugin)

To use the position plugin, add the following lines befor ```</model>``` to ```~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris.sdf``` file

```xml
<plugin name='libposition_plugin' filename='libposition_plugin.so'>
    <body_name>base_link</body_name>
    <update_rate>50</update_rate>
    <xyz_offset>0 0 0</xyz_offset>
    <rpy_offset>0 0 0</rpy_offset>
    <gaussian_noise>0.01</gaussian_noise>
</plugin>
```

## Qualisys - ros2 bridge (MoCap experimental tests)

Clone the repository [qualisys](https://github.com/GiacomoCorradini/qualisys_ros2)

``` bash
git clone https://github.com/GiacomoCorradini/qualisys_ros2.git
```

BUild the repository

```bash
colcon build
. install/setup.bash
```

You need to change the `--server` (ip addr) and `--rate` (hz) arguments in the launch script to match your system.
