# Plugins

## UWB Plugin

To use the uwb gazebo plugin you need to install and build the following repo:

- [rosmsgs](https://github.com/GiacomoCorradini/rosmsgs)
- [UWB gazebo plugin](https://github.com/GiacomoCorradini/uwb_gazebo_plugin)

Then to perform the simulation copy the customize world inside the PX4-Autopilot folder:

```bash
cp ~/PX4-mini-drone/my_world.world ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/
```

To use the uwb plugin, add the following lines befor ```</model>``` to ```~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris.sdf``` file

```xml
<plugin name='libgtec_uwb_plugin' filename='libgtec_uwb_plugin.so'>
    <update_rate>25</update_rate>
    <nlosSoftWallWidth>0.25</nlosSoftWallWidth>
    <tag_z_offset>0</tag_z_offset>
    <tag_link>base_link</tag_link>
    <anchor_prefix>uwb_anchor</anchor_prefix>
    <all_los>false</all_los>
    <tag_id>0</tag_id>
</plugin>
```

<!-- And copy the customize drone model:

```bash
cp -R ~/PX4-mini-drone/my_iris ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/my_iris
``` -->

To lunch the simulation:

```bash
make px4_sitl gazebo-classic_iris PX4_SITL_WORLD=my_world
```
