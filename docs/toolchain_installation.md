# System Details

- Ubuntu 22.04
- ROS2 Humble
- micro-ROS
- Python 3.10

## Set-up

### Install [ROS2-Humble](https://docs.ros.org/en/humble/index.html#)

- [Installation Ubuntu (Debian)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Set locale

``` bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

Setup Sources

``` bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

``` bash
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

``` bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install ROS 2 packages

``` bash
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
```

Environment setup

``` bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
```

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

``` bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```

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

### Installation check, you will need four terminals

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
cd ~/
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
