
[Edit on GitHub](https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/quick_start/quick_start.md "https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/quick_start/quick_start.md") 

Kinetic 
Melodic
Noetic
Dashing
Foxy
Humble
Windows

O : Available  

∆ : Need to check  

X : Unavailable

| Features | Kinetic | Melodic | Noetic | Dashing | Foxy | Galactic | Humble |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Teleop | O | O | O | O | O | O | O |
| SLAM | O | O | O | O | O | O | O |
| Navigation | O | O | O | O | O | O | O |
| Simulation | O | O | O | O | O | O | O |
| Manipulation | O | O | O | O | O | ∆ | O |
| Home Service Challenge | O | O | O | X | X | X | X |
| Autonomous Driving | O | X | O | X | X | X | X |
| Machine Learning | O | O | X | O | X | X | X |

| Examples | Kinetic | Melodic | Noetic | Dashing | Foxy | Galactic | Humble |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Interactive Markers | O | X | X | X | X | X | X |
| Obstacle Detection | O | X | X | O | X | X | X |
| Position Control | O | X | X | O | X | X | X |
| Point Operation | O | X | X | O | X | X | X |
| Patrol | O | X | X | O | X | X | X |
| Follower | O | X | X | X | X | X | X |
| Panorama | O | X | X | X | X | X | X |
| Auto Parking | O | X | X | O | X | X | X |
| Auto Parking(Vision) | O | X | X | X | X | X | X |
| Multi TurtleBot3 | O | X | X | X | X | X | X |

# Quick Start Guide

## [PC Setup](#pc-setup "#pc-setup")

**WARNING**: The contents in this chapter corresponds to the `Remote PC` (your desktop or laptop PC) which will control TurtleBot3. Do not apply this instruction to your TurtleBot3.

**Compatibility WARNING**

* `Raspberry Pi 4` does not support ROS Kinetic.
* `Jetson Nano` does not support ROS Kinetic.

**NOTE**: This instruction was tested on Linux with `Ubuntu 16.04` and `ROS Kinetic Kame`.

### [Download and Install Ubuntu on PC](#download-and-install-ubuntu-on-pc "#download-and-install-ubuntu-on-pc")

1. Download the proper `Ubuntu 16.04 LTS Desktop` image for your PC from the links below.
	* [![](/assets/images/icon_download.png) Ubuntu 16.04 LTS Desktop image (64-bit)](https://releases.ubuntu.com/16.04.7/ "https://releases.ubuntu.com/16.04.7/")
2. Follow the instruction below to install Ubuntu on PC.
	* [Install Ubuntu desktop](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview "https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview")

### [Install ROS on Remote PC](#install-ros-on-remote-pc "#install-ros-on-remote-pc")

Open the terminal with `Ctrl`+`Alt`+`T` and enter below commands one at a time.  

In order to check the details of the easy installation script, please refer to [the script file](https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh "https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh").

```
$ sudo apt-get update
$ sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh
$ chmod 755 ./install_ros_kinetic.sh 
$ bash ./install_ros_kinetic.sh

```

If the above installation fails, please refer to [the official ROS Kinetic installation guide](http://wiki.ros.org/kinetic/Installation/Ubuntu "http://wiki.ros.org/kinetic/Installation/Ubuntu").

### [Install Dependent ROS Packages](#install-dependent-ros-packages "#install-dependent-ros-packages")

```
$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy \
  ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc \
  ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan \
  ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python \
  ros-kinetic-rosserial-server ros-kinetic-rosserial-client \
  ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server \
  ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro \
  ros-kinetic-compressed-image-transport ros-kinetic-rqt\* \
  ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers

```

### [Install TurtleBot3 Packages](#install-turtlebot3-packages "#install-turtlebot3-packages")

Install TurtleBot3 via Debian Packages.

```
$ sudo apt-get install ros-kinetic-dynamixel-sdk
$ sudo apt-get install ros-kinetic-turtlebot3-msgs
$ sudo apt-get install ros-kinetic-turtlebot3

```

![](/assets/images/icon_unfold.png) **Click here to expand more details about building TurtleBot3 package from source.**

In case you need to download the source codes and build them, please use the commands below.  

Make sure to remove the identical packages to avoid redundancy.

```
$ sudo apt-get remove ros-kinetic-dynamixel-sdk
$ sudo apt-get remove ros-kinetic-turtlebot3-msgs
$ sudo apt-get remove ros-kinetic-turtlebot3
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src/
$ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws && catkin_make
$ echo "source ~/catkin\_ws/devel/setup.bash" >> ~/.bashrc

```

### [Set TurtleBot3 Model Name](#set-turtlebot3-model-name "#set-turtlebot3-model-name")

Set the default `TURTLEBOT3_MODEL` name to your model. Enter the below command to a terminal.

* In case of TurtleBot3 Burger

```
$ echo "export TURTLEBOT3\_MODEL=burger" >> ~/.bashrc

```
* In case of TurtleBot3 Waffle Pi

```
$ echo "export TURTLEBOT3\_MODEL=waffle\_pi" >> ~/.bashrc

```

### [Network Configuration](#network-configuration "#network-configuration")

![](/assets/images/platform/turtlebot3/software/network_configuration.png)

1. Connect PC to a WiFi device and find the assigned IP address with the command below.

```
$ ifconfig

```

![](/assets/images/platform/turtlebot3/software/network_configuration2.png)
2. Open the file and update the ROS IP settings with the command below.

```
$ nano ~/.bashrc

```
3. Press `Ctrl`+`END` or `Alt`+`/` to move the cursor to the end of line.  

 Modify the address of `localhost` in the `ROS_MASTER_URI` and `ROS_HOSTNAME` with the IP address acquired from the above terminal window.  

![](/assets/images/platform/turtlebot3/software/network_configuration3.png)
4. Source the bashrc with below command.

```
$ source ~/.bashrc

```

# Quick Start Guide

## [PC Setup](#pc-setup "#pc-setup")

**WARNING**: The contents in this chapter corresponds to the `Remote PC` (your desktop or laptop PC) which will control TurtleBot3. Do not apply this instruction to your TurtleBot3.

**NOTE**: This instruction was tested on Linux with `Ubuntu 18.04` and `ROS1 Melodic Morenia`.

### [Download and Install Ubuntu on PC](#download-and-install-ubuntu-on-pc "#download-and-install-ubuntu-on-pc")

1. Download the proper `Ubuntu 18.04 LTS Desktop` image for your PC from the links below.
	* [Ubuntu 18.04 LTS Desktop image (64-bit)](http://releases.ubuntu.com/18.04/ "http://releases.ubuntu.com/18.04/")
2. Follow the instruction below to install Ubuntu on PC.
	* [Install Ubuntu desktop](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview "https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview")

### [Install ROS on Remote PC](#install-ros-on-remote-pc "#install-ros-on-remote-pc")

Open the terminal with `Ctrl`+`Alt`+`T` and enter below commands one at a time.  

In order to check the details of the easy installation script, please refer to [the script file](https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh "https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh").

```
$ sudo apt update
$ sudo apt upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh
$ chmod 755 ./install_ros_melodic.sh 
$ bash ./install_ros_melodic.sh

```

If the above installation fails, please refer to [the official ROS1 Melodic installation guide](http://wiki.ros.org/melodic/Installation/Ubuntu "http://wiki.ros.org/melodic/Installation/Ubuntu").

### [Install Dependent ROS Packages](#install-dependent-ros-packages "#install-dependent-ros-packages")

```
$ sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
  ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
  ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
  ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
  ros-melodic-rosserial-server ros-melodic-rosserial-client \
  ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
  ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
  ros-melodic-compressed-image-transport ros-melodic-rqt\* \
  ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers

```

### [Install TurtleBot3 Packages](#install-turtlebot3-packages "#install-turtlebot3-packages")

Install TurtleBot3 via Debian Packages.

```
$ sudo apt-get install ros-melodic-dynamixel-sdk
$ sudo apt-get install ros-melodic-turtlebot3-msgs
$ sudo apt-get install ros-melodic-turtlebot3

```

![](/assets/images/icon_unfold.png) **Click here to expand more details about building TurtleBot3 package from source.**

In case you need to download the source codes and build them, please use the commands below.  

Make sure to remove the identical packages to avoid redundancy.

```
$ sudo apt-get remove ros-melodic-dynamixel-sdk
$ sudo apt-get remove ros-melodic-turtlebot3-msgs
$ sudo apt-get remove ros-melodic-turtlebot3
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src/
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws && catkin_make
$ echo "source ~/catkin\_ws/devel/setup.bash" >> ~/.bashrc

```

### [Set TurtleBot3 Model Name](#set-turtlebot3-model-name "#set-turtlebot3-model-name")

Set the default `TURTLEBOT3_MODEL` name to your model. Enter the below command to a terminal.

* In case of TurtleBot3 Burger

```
$ echo "export TURTLEBOT3\_MODEL=burger" >> ~/.bashrc

```
* In case of TurtleBot3 Waffle Pi

```
$ echo "export TURTLEBOT3\_MODEL=waffle\_pi" >> ~/.bashrc

```

### [Network Configuration](#network-configuration "#network-configuration")

![](/assets/images/platform/turtlebot3/software/network_configuration.png)

1. Connect PC to a WiFi device and find the assigned IP address with the command below.

```
$ ifconfig

```

![](/assets/images/platform/turtlebot3/software/network_configuration2.png)
2. Open the file and update the ROS IP settings with the command below.

```
$ nano ~/.bashrc

```
3. Press `Ctrl`+`END` or `Alt`+`/` to move the cursor to the end of line.  

 Modify the address of `localhost` in the `ROS_MASTER_URI` and `ROS_HOSTNAME` with the IP address acquired from the above terminal window.  

![](/assets/images/platform/turtlebot3/software/network_configuration3.png)
4. Source the bashrc with below command.

```
$ source ~/.bashrc

```

# Quick Start Guide

## [PC Setup](#pc-setup "#pc-setup")

**WARNING**: The contents in this chapter corresponds to the `Remote PC` (your desktop or laptop PC) which will control TurtleBot3. Do not apply this instruction to your TurtleBot3.

**Compatibility WARNING**

* `Jetson Nano` does not support native Ubuntu 20.04. Please refer to [NVIDIA developer forum](https://forums.developer.nvidia.com/t/when-will-jetpack-move-to-ubuntu-20-04/142517 "https://forums.developer.nvidia.com/t/when-will-jetpack-move-to-ubuntu-20-04/142517") for more details.

**NOTE**: This instruction was tested on Linux with `Ubuntu 20.04` and `ROS1 Noetic Ninjemys`.

### [Download and Install Ubuntu on PC](#download-and-install-ubuntu-on-pc "#download-and-install-ubuntu-on-pc")

1. Download the proper `Ubuntu 20.04 LTS Desktop` image for your PC from the links below.
	* [Ubuntu 20.04 LTS Desktop image (64-bit)](https://releases.ubuntu.com/20.04/ "https://releases.ubuntu.com/20.04/")
2. Follow the instruction below to install Ubuntu on PC.
	* [Install Ubuntu desktop](https://www.ubuntu.com/download/desktop/install-ubuntu-desktop "https://www.ubuntu.com/download/desktop/install-ubuntu-desktop")

### [Install ROS on Remote PC](#install-ros-on-remote-pc "#install-ros-on-remote-pc")

Open the terminal with `Ctrl`+`Alt`+`T` and enter below commands one at a time.  

In order to check the details of the easy installation script, please refer to [the script file](https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh "https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh").

```
$ sudo apt update
$ sudo apt upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
$ chmod 755 ./install_ros_noetic.sh 
$ bash ./install_ros_noetic.sh

```

If the above installation fails, please refer to [the official ROS1 Noetic installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu "http://wiki.ros.org/noetic/Installation/Ubuntu").

### [Install Dependent ROS Packages](#install-dependent-ros-packages "#install-dependent-ros-packages")

```
$ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt\* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

```

### [Install TurtleBot3 Packages](#install-turtlebot3-packages "#install-turtlebot3-packages")

Install TurtleBot3 via Debian Packages.

```
$ sudo apt install ros-noetic-dynamixel-sdk
$ sudo apt install ros-noetic-turtlebot3-msgs
$ sudo apt install ros-noetic-turtlebot3

```

![](/assets/images/icon_unfold.png) **Click here to expand more details about building TurtleBot3 package from source.**

In case you need to download the source codes and build them, please use the commands below.  

Make sure to remove the identical packages to avoid redundancy.

```
$ sudo apt remove ros-noetic-dynamixel-sdk
$ sudo apt remove ros-noetic-turtlebot3-msgs
$ sudo apt remove ros-noetic-turtlebot3
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws && catkin_make
$ echo "source ~/catkin\_ws/devel/setup.bash" >> ~/.bashrc

```

### [Network Configuration](#network-configuration "#network-configuration")

![](/assets/images/platform/turtlebot3/software/network_configuration.png)

1. Connect PC to a WiFi device and find the assigned IP address with the command below.

```
$ ifconfig

```

![](/assets/images/platform/turtlebot3/software/network_configuration2.png)
2. Open the file and update the ROS IP settings with the command below.

```
$ nano ~/.bashrc

```
3. Press `Ctrl`+`END` or `Alt`+`/` to move the cursor to the end of line.  

 Modify the address of `localhost` in the `ROS_MASTER_URI` and `ROS_HOSTNAME` with the IP address acquired from the above terminal window.  

![](/assets/images/platform/turtlebot3/software/network_configuration3.png)
4. Source the bashrc with below command.

```
$ source ~/.bashrc

```

# Quick Start Guide

## [PC Setup](#pc-setup "#pc-setup")

**WARNING**: The contents in this chapter corresponds to the `Remote PC` (your desktop or laptop PC) which will control TurtleBot3. Do not apply this instruction to your TurtleBot3.

**NOTE**: This instruction was tested on Linux with `Ubuntu 18.04` and `ROS2 Dashing Diademata`.

### [Download and Install Ubuntu on PC](#download-and-install-ubuntu-on-pc "#download-and-install-ubuntu-on-pc")

1. Download the proper `Ubuntu 18.04 LTS Desktop` image for your PC from the links below.
	* [Ubuntu 18.04 LTS Desktop image (64-bit)](http://releases.ubuntu.com/18.04/ "http://releases.ubuntu.com/18.04/")
2. Follow the instruction below to install Ubuntu on PC.
	* [Install Ubuntu desktop](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview "https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview")

### [Install ROS 2 on Remote PC](#install-ros-2-on-remote-pc "#install-ros-2-on-remote-pc")

Open the terminal with `Ctrl`+`Alt`+`T` and enter below commands one at a time.  

In order to check the details of the easy installation script, please refer to [the script file](https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_dashing.sh "https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_dashing.sh").

```
$ sudo apt-get update
$ sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_dashing.sh
$ chmod 755 ./install_ros2_dashing.sh
$ bash ./install_ros2_dashing.sh

```

If the above installation fails, please refer to [the official ROS2 Dashing installation guide](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/ "https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/").

### [Install Dependent ROS 2 Packages](#install-dependent-ros-2-packages "#install-dependent-ros-2-packages")

1. Open the terminal with `Ctrl`+`Alt`+`T` from **Remote PC**.
2. Install Colcon

```
$ sudo apt install python3-colcon-common-extensions

```
3. Install Gazebo9

```
$ curl -sSL http://get.gazebosim.org | sh

```
4. Uninstall Gazebo11 if installed previously

```
$ sudo apt remove gazebo11 libgazebo11-dev
$ sudo apt install gazebo9 libgazebo9-dev
$ sudo apt install ros-dashing-gazebo-ros-pkgs

```
5. Install Cartographer

```
$ sudo apt install ros-dashing-cartographer
$ sudo apt install ros-dashing-cartographer-ros

```
6. Install Navigation2

```
$ sudo apt install ros-dashing-navigation2
$ sudo apt install ros-dashing-nav2-bringup

```
7. Install vcstool

```
$ sudo apt install python3-vcstool

```

### [Install TurtleBot3 Packages](#install-turtlebot3-packages "#install-turtlebot3-packages")

Install TurtleBot3 via Debian Packages.

```
$ source /opt/ros/dashing/setup.bash
$ sudo apt install ros-dashing-dynamixel-sdk
$ sudo apt install ros-dashing-turtlebot3-msgs
$ sudo apt install ros-dashing-turtlebot3

```

![](/assets/images/icon_unfold.png) **Click here to expand more details about building TurtleBot3 package from source.**

In case you need to build the TurtleBot3 packages with source code, please use the commands below.  

Building the source code provides most up to date contents which may have resolved known issues.  

Make sure to remove the binary packages to avoid redundancy.

```
$ sudo apt remove ros-dashing-turtlebot3-msgs
$ sudo apt remove ros-dashing-turtlebot3
$ mkdir -p ~/turtlebot3_ws/src
$ cd ~/turtlebot3_ws/src/
$ git clone -b dashing-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ colcon build --symlink-install
$ source ~/.bashrc

```

### [Environment Configuration](#environment-configuration "#environment-configuration")

1. Set the ROS environment for PC.

```
$ echo 'source ~/turtlebot3\_ws/install/setup.bash' >> ~/.bashrc
$ echo 'export ROS\_DOMAIN\_ID=30 #TURTLEBOT3' >> ~/.bashrc
$ source ~/.bashrc

```

If you have installed TurtleBot3 using `apt install` command, you can ignore the warning below.

```
bash: /home/{$YOUR\_ACCOUNT}/turtlebot3_ws/install/setup.bash: No such file or directory

```

# Quick Start Guide

## [PC Setup](#pc-setup "#pc-setup")

**WARNING**: The contents in this chapter corresponds to the `Remote PC` (your desktop or laptop PC) which will control TurtleBot3. Do not apply this instruction to your TurtleBot3.

**Compatibility WARNING**

* `Jetson Nano` does not support Ubuntu 20.04 and later. Please refer to [NVIDIA developer forum](https://forums.developer.nvidia.com/t/jetpack-5-0-2/223564/2 "https://forums.developer.nvidia.com/t/jetpack-5-0-2/223564/2") for more details.

**NOTE**: This instruction was tested on Linux with `Ubuntu 20.04` and `ROS2 Foxy Fitzroy`.

### [Download and Install Ubuntu on PC](#download-and-install-ubuntu-on-pc "#download-and-install-ubuntu-on-pc")

1. Download the proper `Ubuntu 20.04 LTS Desktop` image for your PC from the links below.
	* [Ubuntu 20.04 LTS Desktop image (64-bit)](https://releases.ubuntu.com/20.04/ "https://releases.ubuntu.com/20.04/")
2. Follow the instruction below to install Ubuntu on PC.
	* [Install Ubuntu desktop](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview "https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview")

### [Install ROS 2 on Remote PC](#install-ros-2-on-remote-pc "#install-ros-2-on-remote-pc")

Open the terminal with `Ctrl`+`Alt`+`T` and enter below commands one at a time.  

In order to check the details of the easy installation script, please refer to [the script file](https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh "https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh").

```
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh
$ sudo chmod 755 ./install_ros2_foxy.sh
$ bash ./install_ros2_foxy.sh

```

If the above installation fails, please refer to [the official ROS2 Foxy installation guide](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/ "https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/").

### [Install Dependent ROS 2 Packages](#install-dependent-ros-2-packages "#install-dependent-ros-2-packages")

1. Open the terminal with `Ctrl`+`Alt`+`T` from **Remote PC**.
2. Install Gazebo11

```
$ sudo apt-get install ros-foxy-gazebo-\*

```
3. Install Cartographer

```
$ sudo apt install ros-foxy-cartographer
$ sudo apt install ros-foxy-cartographer-ros

```
4. Install Navigation2

```
$ sudo apt install ros-foxy-navigation2
$ sudo apt install ros-foxy-nav2-bringup

```

### [Install TurtleBot3 Packages](#install-turtlebot3-packages "#install-turtlebot3-packages")

Install TurtleBot3 via Debian Packages.

```
$ source ~/.bashrc
$ sudo apt install ros-foxy-dynamixel-sdk
$ sudo apt install ros-foxy-turtlebot3-msgs
$ sudo apt install ros-foxy-turtlebot3

```

![](/assets/images/icon_unfold.png) **Click here to expand more details about building TurtleBot3 package from source.**

In case you need to build the TurtleBot3 packages with source code, please use the commands below.  

Building the source code provides most up to date contents which may have resolved known issues.  

Make sure to remove the binary packages to avoid redundancy.

```
$ sudo apt remove ros-foxy-turtlebot3-msgs
$ sudo apt remove ros-foxy-turtlebot3
$ mkdir -p ~/turtlebot3_ws/src
$ cd ~/turtlebot3_ws/src/
$ git clone -b foxy-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/turtlebot3_ws
$ colcon build --symlink-install
$ echo 'source ~/turtlebot3\_ws/install/setup.bash' >> ~/.bashrc
$ source ~/.bashrc

```

### [Environment Configuration](#environment-configuration "#environment-configuration")

1. Set the ROS environment for PC.

```
$ echo 'export ROS\_DOMAIN\_ID=30 #TURTLEBOT3' >> ~/.bashrc
$ source ~/.bashrc

```

If you have installed TurtleBot3 using Debian packages with `apt install` command, you can ignore the warning below.

```
bash: /home/{$YOUR\_ACCOUNT}/turtlebot3_ws/install/setup.bash: No such file or directory

```

# Quick Start Guide

## [PC Setup](#pc-setup "#pc-setup")

**WARNING**: The contents in this chapter corresponds to the `Remote PC` (your desktop or laptop PC) which will control TurtleBot3. Do not apply this instruction to your TurtleBot3.

**Compatibility WARNING**

* `Jetson Nano` does not support Ubuntu 20.04 and later. Please refer to [NVIDIA developer forum](https://forums.developer.nvidia.com/t/jetpack-5-0-2/223564/2 "https://forums.developer.nvidia.com/t/jetpack-5-0-2/223564/2") for more details.

**NOTE**: This instruction was tested on Linux with `Ubuntu 22.04` and `ROS2 Humble Hawksbill`.

### [Download and Install Ubuntu on PC](#download-and-install-ubuntu-on-pc "#download-and-install-ubuntu-on-pc")

1. Download the proper `Ubuntu 22.04 LTS Desktop` image for your PC from the links below.
	* [Ubuntu 22.04 LTS Desktop image (64-bit)](https://releases.ubuntu.com/22.04/ "https://releases.ubuntu.com/22.04/")
2. Follow the instruction below to install Ubuntu on PC.
	* [Install Ubuntu desktop](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview "https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview")

### [Install ROS 2 on Remote PC](#install-ros-2-on-remote-pc "#install-ros-2-on-remote-pc")

Please follow [the official ROS2 documentation](https://docs.ros.org/en/humble/Installation.html "https://docs.ros.org/en/humble/Installation.html") to install the ROS2 Humble.  

For most Linux users, [Debian package installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html "https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html") method is strongly recommended.

### [Install Dependent ROS 2 Packages](#install-dependent-ros-2-packages "#install-dependent-ros-2-packages")

1. Open the terminal with `Ctrl`+`Alt`+`T` from **Remote PC**.
2. Install Gazebo

```
$ sudo apt install ros-humble-gazebo-\*

```
3. Install Cartographer

```
$ sudo apt install ros-humble-cartographer
$ sudo apt install ros-humble-cartographer-ros

```
4. Install Navigation2

```
$ sudo apt install ros-humble-navigation2
$ sudo apt install ros-humble-nav2-bringup

```

### [Install TurtleBot3 Packages](#install-turtlebot3-packages "#install-turtlebot3-packages")

Install TurtleBot3 via Debian Packages.

```
$ source ~/.bashrc
$ sudo apt install ros-humble-dynamixel-sdk
$ sudo apt install ros-humble-turtlebot3-msgs
$ sudo apt install ros-humble-turtlebot3

```

![](/assets/images/icon_unfold.png) **Click here to expand more details about building TurtleBot3 package from source.**

In case you need to build the TurtleBot3 packages with source code, please use the commands below.  

Building the source code provides most up to date contents which may have resolved known issues.  

Make sure to remove the binary packages to avoid redundancy.

```
$ sudo apt remove ros-humble-turtlebot3-msgs
$ sudo apt remove ros-humble-turtlebot3
$ mkdir -p ~/turtlebot3_ws/src
$ cd ~/turtlebot3_ws/src/
$ git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/turtlebot3_ws
$ colcon build --symlink-install
$ echo 'source ~/turtlebot3\_ws/install/setup.bash' >> ~/.bashrc
$ source ~/.bashrc

```

### [Environment Configuration](#environment-configuration "#environment-configuration")

1. Set the ROS environment for PC.

```
$ echo 'export ROS\_DOMAIN\_ID=30 #TURTLEBOT3' >> ~/.bashrc
$ source ~/.bashrc

```

If you have installed TurtleBot3 using Debian packages with `apt install` command, you can ignore the warning below.

```
bash: /home/{$YOUR\_ACCOUNT}/turtlebot3_ws/install/setup.bash: No such file or directory

```

# Quick Start Guide

## [PC Setup](#pc-setup "#pc-setup")

**NOTE**: This instruction was tested on Windows with `Windows 10 IoT Enterprise` and `ROS1 Melodic Morenia` or `ROS1 Noetic Ninjemys`.

**WARNING**

* This SBC Setup section for Windows is tested with `Intel Up²` or `Intel NUC` on TurtleBot3.
* This SBC will be operating as Remote PC and SBC at the same time.
* This process may take long time. Please do not use battery while following this section.
* An HDMI monitor and input devices such as a keyboard and a mouse will be required to complete this setup.

### [Setup Remote PC](#setup-remote-pc "#setup-remote-pc")

1. If you do not already have `Windows 10` on your Remote PC (Desktop, Laptop or SBC), you can download a trial of Windows 10 IoT Enterprise from the following link:
	* [Download Windows 10 IoT Enterprise LTSC(Trial)](https://www.microsoft.com/en-us/evalcenter/evaluate-windows-10-enterprise "https://www.microsoft.com/en-us/evalcenter/evaluate-windows-10-enterprise")
2. Please refer to the [ROS Wiki instructions](https://wiki.ros.org/Installation/Windows "https://wiki.ros.org/Installation/Windows") for installing ROS on Windows.
3. Install LDS-01 Lidar driver
	* [CP2102 Driver](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers "https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers")
4. Create workspace and download TurtleBot3 packages.  

 For more details, please refer to [Microsoft Instructions](https://ms-iot.github.io/ROSOnWindows/Turtlebot/Turtlebot3.html "https://ms-iot.github.io/ROSOnWindows/Turtlebot/Turtlebot3.html").

```
> mkdir c:\ws\turtlebot3\src
> cd c:\ws\turtlebot3
> curl -o tb3.repos https://raw.githubusercontent.com/ms-iot/ROSOnWindows/master/docs/Turtlebot/turtlebot3_ros1.repos
> vcs import src < tb3.repos

```
5. Customize TurtleBot3 Launch Files.  

 Modify the ROS Launch files to map the devices to the correct COM port. To determine which COM ports you require, right click on the Windows Start Menu, Select Device Manager.
	* Under the `Ports (COM & LPT)` node:
		+ USB Serial Device (COMx) is the OpenCR board.
		+ Silicon Labs CP210x USB to UART Bridge (COMy) is the Lidar.
	* Enter the COM port in the correct fields in the launch files below:

		+ turtlebot3\_bringup/launch/turtlebot3\_core.launch

```
	   <node pkg="rosserial_python" type="serial_node.py" name="turtlebot3_core" output="screen">
	       <param name="port" value="COMx"/>

```

		+ turtlebot3\_bringup/launch/turtlebot3\_lidar.launch

```
	   <node pkg="hls_lfcd_lds_driver" type="hlds_laser_publisher" name="turtlebot3_lds" output="screen">
	       <param name="port" value="COMy"/>

```
6. Build the Workspace

```
> cd c:\ws\turtlebot3
> catkin_make
> c:\ws\turtlebot3\devel\setup.bat

```

### [Network Configuration](#network-configuration "#network-configuration")

To communicate from a Windows 10 system to a remote single board computer (SBC) running on the turtlebot, set the following environment variables:

```
> set ROS_MASTER_URI=http://<IP address of the PC>:11311
> set ROS_HOSTNAME=<name of the windows computer>

```

 Previous Page
Next Page 
