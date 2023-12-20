# System Details

- Python 3.10
- Ubuntu 22.04
- ROS2 Humble
- MicroXRCE

## Set-up

## Clone the repository and build the code

```bash
git clone https://github.com/mattiapettene/PX4-mini-drone.git
colcon build
source ~/PX4-mini-drone/install/setup.bash && echo "source ~/PX4-mini-drone/install/setup.bash" >> .bashrc
```

### Install [ROS2-Humble](https://docs.ros.org/en/humble/index.html#)

- [Installation Ubuntu (Debian)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

``` bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
```

- Some Python dependencies must also be installed:

```bash
pip install --user -U empy pyros-genmsg setuptools
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

- Install gazebo classic

```bash
sudo apt install gazebo
sudo apt-get install ros-humble-gazebo-ros-pkgs
```

### Install PX4 ROS Com and PX4 msg

- Download the repositories

``` bash
cd ~
mkdir -p ~/ws_combined_sensor/src/
cd ~/ws_combined_sensor/src/
```

``` bash
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git
```

- Build them

``` bash
cd ~/ws_combined_sensor/
colcon build
source ~/ws_combined_sensor/install/setup.bash && echo "source ~/ws_combined_sensor/install/setup.bash" >> .bashrc
```

### PX4-FastDDS Bridge

PX4 uses XRCE-DDS middleware to allow uORB messages to be published and subscribed on a companion computer as though they were ROS 2 topics.

**[Install MicroXRCE agent](https://micro-xrce-dds.docs.eprosima.com/en/stable/agent.html)**

``` bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

- Try running the agent

``` bash
MicroXRCEAgent udp4 -p 8888
```

## Installation check, you will need four terminals

- Install the [px4-offboard](https://github.com/Jaeyoung-Lim/px4-offboard) example

``` bash
cd ~
git clone https://github.com/Jaeyoung-Lim/px4-offboard.git
```

``` bash
cd ~/px4-offboard
colcon build
```

- Start the Micro-XRCE-DDS-Agent

``` bash
cd ~/Micro-XRCE-DDS-Agent
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
MicroXRCEAgent udp4 -p 8888
```

- Start Gazebo

``` bash
cd ~/PX4-Autopilot
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
make px4_sitl gazebo-classic
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
