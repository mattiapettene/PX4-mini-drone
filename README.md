# PX4-mini-drone

## Collaborators

- [Giacomo Corradini](https://github.com/GiacomoCorradini)
- [Mattia Pettene](https://github.com/mattiapettene)

## Example System Details

- Ubuntu 22.04
- ROS2 Humble
- micro-ROS
- Python 3.10

## Set-up

### Install [ROS2-Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

- Installation with Ubuntu (Debian)
- Additional packages

``` bash
sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools
```

Install packages according to your Ubuntu version (Ubuntu 22.04)

``` bash
sudo apt install -y \
   python3-flake8-blind-except \
   python3-flake8-builtins \
   python3-flake8-class-newline \
   python3-flake8-comprehensions \
   python3-flake8-deprecated \
   python3-flake8-import-order \
   python3-flake8-quotes \
   python3-pytest-repeat \
   python3-pytest-rerunfailures
```

- Configuration environment step ([Tutorial section](https://docs.ros.org/en/humble/Tutorials.html))

- Initializing rosdep

``` bash
sudo rosdep init
rosdep update
```

### Install [QGroundControl](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)

- On the command prompt enter

``` bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
```

- Logout and login again to enable the change to user permissions

To install QGroundControl

- Download [QGroundControl.AppImage](https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage)
- Install (and run) using the terminal commands

``` bash
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage
```

### Download and install PX4 Source code

- The PX4 source code is stored on Github in the [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) repository

``` bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

- Install common packages

``` bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

### Install PX4 ROS Com and PX4 msg

- Download the repositories

``` bash
cd ~
mkdir px4_ros_com_ws
cd px4_ros_com_ws
```

``` bash
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git
```

- Build them

``` bash
colcon build
```

### Install micro-ROS agent

- Ensure your ROS_DISTRO environment variable is set

``` bash
export ROS_DISTRO=humble
```

- Building the [micro-ROS-agent](https://github.com/micro-ROS/micro_ros_setup#building-micro-ros-agent)

``` bash
cd ~
mkdir ~/microros_ws && cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git 
```

``` bash
mkdir src
mv micro_ros_setup/ src/
colcon build
```

``` bash
source install/local_setup.sh
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

- Try running the agent

``` bash
source install/local_setup.sh
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### Installation check

- Install the [px4-offboard](https://github.com/Jaeyoung-Lim/px4-offboard) example

``` bash
cd ~
git clone https://github.com/Jaeyoung-Lim/px4-offboard.git
```

``` bash
cd ~/px4-offboard
colcon build
```

- Start the micro-ros-agent

``` bash
cd ~/microros_ws
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
source install/local_setup.sh
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 ROS_DOMAIN_ID=0
```

- Start Gazebo

``` bash
cd ~/PX4-Autopilot
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
make px4_sitl gazebo
```

- Start QGround Controller and Take Off

``` bash
./QGroundControl.AppImage
```

- Press the takeoff command on QGround Controller interface, the simulated drone should takeoff and climb to an altitude of 10m (~32ft)

- Start the px4-offboard example

``` bash
cd ~/px4-offboard
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
source ../px4_ros_com_ws/install/setup.bash
source install/setup.bash
ros2 launch px4_offboard offboard_position_control.launch.py
```

- Now head back to QGround Controller and enable offboard control

- After a 1-2 sec pause, the demo should start
