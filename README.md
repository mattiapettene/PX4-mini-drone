# PX4-mini-drone

### Example System Details

- Ubuntu 22.02
- ROS2 Humble
- micro-ROS
- Python 3.10

### Set-up

1) **Install ROS2 - Humble** ([ROS2-Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html))

2) **Install QGroundControl** ([QGroundControl](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html))

- On the command prompt enter:
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
```
- Logout and login again to enable the change to user permissions.

To install QGroundControl:

- Download QGroundControl.AppImage. ([QGroundControl.AppImage](https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage))
- Install (and run) using the terminal commands:

```
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage
```
3) **Download and install PX4 Source code**

The PX4 source code is stored on Github in the [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) repository.
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```
```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

4) **Install PX4 ROS Com and PX4 msg**

Download the repositories
```
cd ~
mkdir px4_ros_com_ws
cd px4_ros_com_ws
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git
```

Build them:
```
colcon build
```

5) **Install micro-ROS agent**

Ensure your ROS_DISTRO environment variable is set
```
env | grep ROS
```
You should see this variable with some other ROS settings:
```
ROS_DISTRO=humble
```
If it's not set you can try to update:
```
export ROS_DISTRO=humble
```
- Building the micro_ros_agent (https://github.com/micro-ROS/micro_ros_setup#building-micro-ros-agent)
```
cd ~
mkdir ~/microros_ws && cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git 
mkdir src
mv micro_ros_setup/ src/
colcon build
source install/local_setup.sh
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

Try running the agent
```
source install/local_setup.sh
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

6) **Installation check**

Try the [px4-offboard](https://github.com/Jaeyoung-Lim/px4-offboard) example to check the installation

- TO DO