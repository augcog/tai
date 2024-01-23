
[Edit on GitHub](https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/simulation/simulation.md "https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/simulation/simulation.md") 

Kinetic 
Melodic
Noetic
Dashing
Foxy
Humble
Windows

# [Simulation](#simulation "#simulation")

**NOTE**

* Please run the Simulation on **Remote PC**.
* Launching the Simulation for the first time on the Remote PC may take a while to setup the environment.

![](/assets/images/icon_unfold.png) Read more about **TurtleBot3 Simulation**

TurtleBot3 supports simulation development environment that can be programmed and developed with a virtual robot in the simulation. There are two development environments to do this, one is using a **fake node with 3D visualization tool RViz**, and the other is using the **3D robot simulator Gazebo**.

* The **fake node** is suitable for testing with the robot model and movement, but it does not support sensors.
* If you need to perform SLAM or Navigation, **Gazebo** would be a feasible solution as it supports sensors such as IMU, LDS, and camera.

In this instruction, Gazebo will be mainly introduced which is most widely used among ROS developers.

* **Gazebo Tutorials** : [http://gazebosim.org/tutorials](http://gazebosim.org/tutorials "http://gazebosim.org/tutorials")

## [Gazebo Simulation](#gazebo-simulation "#gazebo-simulation")

The contents in e-Manual can be updated without a prior notice and video contents could be outdated.

This Gazebo Simulation uses **ROS Gazebo package**, therefore, proper Gazebo version for ROS1 Kinetic has to be installed before running this instruction.

### [Install Simulation Package](#install-simulation-package "#install-simulation-package")

The **TurtleBot3 Simulation Package** requires `turtlebot3` and `turtlebot3_msgs` packages as prerequisite. Without these prerequisite packages, the Simulation cannot be launched.  

Please follow the [PC Setup](/docs/en/platform/turtlebot3/quick-start/ "/docs/en/platform/turtlebot3/quick-start/") instructions if you did not install required packages and dependent packages.

```
$ cd ~/catkin_ws/src/
$ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make

```

### [Launch Simulation World](#launch-simulation-world "#launch-simulation-world")

Three simulation environments are prepared for TurtleBot3. Please select one of these environments to launch Gazebo.

**Please make sure to completely terminate other Simulation world before launching a new world.**

1. Empty World  

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_empty_world.png)

```
$ export TURTLEBOT3\_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

```
2. TurtleBot3 World  

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_world_bugger.png)

```
$ export TURTLEBOT3\_MODEL=waffle
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch

```
3. TurtleBot3 House  

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_house.png)

```
$ export TURTLEBOT3\_MODEL=waffle_pi
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch

```

**NOTE**: If TurtleBot3 House is launched for the first time, downloading the map may take more than a few minutes depending the network status.

### [Operate TurtleBot3](#operate-turtlebot3 "#operate-turtlebot3")

In order to teleoperate the TurtleBot3 with the keyboard, launch the teleoperation node with below command in a new terminal window.

```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

```

![](/assets/images/icon_unfold.png) Read more about **How to run Autonomous Collision Avoidance**

A simple collision avoidance node is prepared which keeps certain distance from obstacles and make turns to avoid collision.  

In order to autonomously drive a TurtleBot3 in the **TurtleBot3 world**, please follow the instruction below.

1. Terminate the turtlebot3\_teleop\_key node by entering `Ctrl` + `C` to the terminal that runs the teleop node.
2. Enter the below command to the terminal.

```
$ roslaunch turtlebot3_gazebo turtlebot3_simulation.launch

```

![](/assets/images/icon_unfold.png) Read more about **How to visualize Simulation data(RViz)**

RViz visualizes published topics while simulation is running. You can launch RViz in a new terminal window by entering below command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

```

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_gazebo_rviz.png)

**NOTE**

* Please run the Simulation on **Remote PC**.
* Launching the Simulation for the first time on the Remote PC may take a while to setup the environment.

![](/assets/images/icon_unfold.png) Read more about **TurtleBot3 Simulation**

TurtleBot3 supports simulation development environment that can be programmed and developed with a virtual robot in the simulation. There are two development environments to do this, one is using a **fake node with 3D visualization tool RViz**, and the other is using the **3D robot simulator Gazebo**.

* The **fake node** is suitable for testing with the robot model and movement, but it does not support sensors.
* If you need to perform SLAM or Navigation, **Gazebo** would be a feasible solution as it supports sensors such as IMU, LDS, and camera.

In this instruction, Gazebo will be mainly introduced which is most widely used among ROS developers.

* **Gazebo Tutorials** : [http://gazebosim.org/tutorials](http://gazebosim.org/tutorials "http://gazebosim.org/tutorials")

## [Gazebo Simulation](#gazebo-simulation "#gazebo-simulation")

The contents in e-Manual can be updated without a prior notice and video contents could be outdated.

This Gazebo Simulation uses **ROS Gazebo package**, therefore, proper Gazebo version for ROS1 Melodic has to be installed before running this instruction.

### [Install Simulation Package](#install-simulation-package "#install-simulation-package")

The **TurtleBot3 Simulation Package** requires `turtlebot3` and `turtlebot3_msgs` packages as prerequisite. Without these prerequisite packages, the Simulation cannot be launched.  

Please follow the [PC Setup](/docs/en/platform/turtlebot3/quick-start/ "/docs/en/platform/turtlebot3/quick-start/") instructions if you did not install required packages and dependent packages.

```
$ cd ~/catkin_ws/src/
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make

```

### [Launch Simulation World](#launch-simulation-world "#launch-simulation-world")

Three simulation environments are prepared for TurtleBot3. Please select one of these environments to launch Gazebo.

**Please make sure to completely terminate other Simulation world before launching a new world.**

1. Empty World  

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_empty_world.png)

```
$ export TURTLEBOT3\_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

```
2. TurtleBot3 World  

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_world_bugger.png)

```
$ export TURTLEBOT3\_MODEL=waffle
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch

```
3. TurtleBot3 House  

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_house.png)

```
$ export TURTLEBOT3\_MODEL=waffle_pi
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch

```

**NOTE**: If TurtleBot3 House is launched for the first time, downloading the map may take more than a few minutes depending the network status.

### [Operate TurtleBot3](#operate-turtlebot3 "#operate-turtlebot3")

In order to teleoperate the TurtleBot3 with the keyboard, launch the teleoperation node with below command in a new terminal window.

```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

```

![](/assets/images/icon_unfold.png) Read more about **How to run Autonomous Collision Avoidance**

A simple collision avoidance node is prepared which keeps certain distance from obstacles and make turns to avoid collision.  

In order to autonomously drive a TurtleBot3 in the **TurtleBot3 world**, please follow the instruction below.

1. Terminate the turtlebot3\_teleop\_key node by entering `Ctrl` + `C` to the terminal that runs the teleop node.
2. Enter the below command to the terminal.

```
$ roslaunch turtlebot3_gazebo turtlebot3_simulation.launch

```

![](/assets/images/icon_unfold.png) Read more about **How to visualize Simulation data(RViz)**

RViz visualizes published topics while simulation is running. You can launch RViz in a new terminal window by entering below command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

```

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_gazebo_rviz.png)

**NOTE**

* Please run the Simulation on **Remote PC**.
* Launching the Simulation for the first time on the Remote PC may take a while to setup the environment.

![](/assets/images/icon_unfold.png) Read more about **TurtleBot3 Simulation**

TurtleBot3 supports simulation development environment that can be programmed and developed with a virtual robot in the simulation. There are two development environments to do this, one is using a **fake node with 3D visualization tool RViz**, and the other is using the **3D robot simulator Gazebo**.

* The **fake node** is suitable for testing with the robot model and movement, but it does not support sensors.
* If you need to perform SLAM or Navigation, **Gazebo** would be a feasible solution as it supports sensors such as IMU, LDS, and camera.

In this instruction, Gazebo will be mainly introduced which is most widely used among ROS developers.

* **Gazebo Tutorials** : [http://gazebosim.org/tutorials](http://gazebosim.org/tutorials "http://gazebosim.org/tutorials")

## [Gazebo Simulation](#gazebo-simulation "#gazebo-simulation")

The contents in e-Manual can be updated without a prior notice and video contents could be outdated.

This Gazebo Simulation uses **ROS Gazebo package**, therefore, proper Gazebo version for ROS1 Noetic has to be installed before running this instruction.

### [Install Simulation Package](#install-simulation-package "#install-simulation-package")

The **TurtleBot3 Simulation Package** requires `turtlebot3` and `turtlebot3_msgs` packages as prerequisite. Without these prerequisite packages, the Simulation cannot be launched.  

Please follow the [PC Setup](/docs/en/platform/turtlebot3/quick-start/ "/docs/en/platform/turtlebot3/quick-start/") instructions if you did not install required packages and dependent packages.

```
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make

```

### [Launch Simulation World](#launch-simulation-world "#launch-simulation-world")

Three simulation environments are prepared for TurtleBot3. Please select one of these environments to launch Gazebo.

**Please make sure to completely terminate other Simulation world before launching a new world.**

1. Empty World  

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_empty_world.png)

```
$ export TURTLEBOT3\_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

```
2. TurtleBot3 World  

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_world_bugger.png)

```
$ export TURTLEBOT3\_MODEL=waffle
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch

```
3. TurtleBot3 House  

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_house.png)

```
$ export TURTLEBOT3\_MODEL=waffle_pi
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch

```

**NOTE**: If TurtleBot3 House is launched for the first time, downloading the map may take more than a few minutes depending the network status.

### [Operate TurtleBot3](#operate-turtlebot3 "#operate-turtlebot3")

In order to teleoperate the TurtleBot3 with the keyboard, launch the teleoperation node with below command in a new terminal window.

```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

```

![](/assets/images/icon_unfold.png) Read more about **How to run Autonomous Collision Avoidance**

A simple collision avoidance node is prepared which keeps certain distance from obstacles and make turns to avoid collision.  

In order to autonomously drive a TurtleBot3 in the **TurtleBot3 world**, please follow the instruction below.

1. Terminate the turtlebot3\_teleop\_key node by entering `Ctrl` + `C` to the terminal that runs the teleop node.
2. Enter the below command to the terminal.

```
$ roslaunch turtlebot3_gazebo turtlebot3_simulation.launch

```

![](/assets/images/icon_unfold.png) Read more about **How to visualize Simulation data(RViz)**

RViz visualizes published topics while simulation is running. You can launch RViz in a new terminal window by entering below command.

```
$ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

```

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_gazebo_rviz.png)

**NOTE**

* Please run the Simulation on **Remote PC**.
* Launching the Simulation for the first time on the Remote PC may take a while to setup the environment.

![](/assets/images/icon_unfold.png) Read more about **TurtleBot3 Simulation**

TurtleBot3 supports simulation development environment that can be programmed and developed with a virtual robot in the simulation. There are two development environments to do this, one is using a **fake node with 3D visualization tool RViz**, and the other is using the **3D robot simulator Gazebo**.

* The **fake node** is suitable for testing with the robot model and movement, but it does not support sensors.
* If you need to perform SLAM or Navigation, **Gazebo** would be a feasible solution as it supports sensors such as IMU, LDS, and camera.

In this instruction, Gazebo will be mainly introduced which is most widely used among ROS developers.

* **Gazebo Tutorials** : [http://gazebosim.org/tutorials](http://gazebosim.org/tutorials "http://gazebosim.org/tutorials")

## [Gazebo Simulation](#gazebo-simulation "#gazebo-simulation")

The contents in e-Manual can be updated without a prior notice and video contents could be outdated.

This Gazebo Simulation uses **ROS Gazebo package**, therefore, proper Gazebo version for ROS2 Dashing has to be installed before running this instruction.

### [Install Simulation Package](#install-simulation-package "#install-simulation-package")

The **TurtleBot3 Simulation Package** requires `turtlebot3` and `turtlebot3_msgs` packages as prerequisite. Without these prerequisite packages, the Simulation cannot be launched.  

Please follow the [PC Setup](/docs/en/platform/turtlebot3/quick-start/ "/docs/en/platform/turtlebot3/quick-start/") instructions if you did not install required packages and dependent packages.

```
$ cd ~/turtlebot3_ws/src/
$ git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/turtlebot3_ws && colcon build --symlink-install

```

The **GAZEBO\_MODEL\_PATH** parameter must be appended in the `.bashrc` file. Enter below command to add the information.

```
$ echo 'export GAZEBO\_MODEL\_PATH=$GAZEBO\_MODEL\_PATH:~/turtlebot3\_ws/src/turtlebot3/turtlebot3\_simulations/turtlebot3\_gazebo/models' >> ~/.bashrc
$ source ~/.bashrc

```

### [Launch Simulation World](#launch-simulation-world "#launch-simulation-world")

Three simulation environments are prepared for TurtleBot3. Please select one of these environments to launch Gazebo.

**Please make sure to completely terminate other Simulation world before launching a new world.**

1. Empty World  

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_empty_world.png)

```
$ export TURTLEBOT3\_MODEL=burger
$ ros2 launch turtlebot3_gazebo empty_world.launch.py

```
2. TurtleBot3 World  

![](/assets/images/platform/turtlebot3/ros2/gazebo_world.png)

```
$ export TURTLEBOT3\_MODEL=waffle
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

```
3. TurtleBot3 House  

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_house.png)

```
$ export TURTLEBOT3\_MODEL=waffle_pi
$ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

```

**NOTE**: If TurtleBot3 House is launched for the first time, downloading the map may take more than a few minutes depending the network status.

### [Operate TurtleBot3](#operate-turtlebot3 "#operate-turtlebot3")

In order to teleoperate the TurtleBot3 with the keyboard, launch the teleoperation node with below command in a new terminal window.

```
$ ros2 run turtlebot3_teleop teleop_keyboard

```

![](/assets/images/icon_unfold.png) Read more about **How to run Autonomous Collision Avoidance**

A simple collision avoidance node is prepared which keeps certain distance from obstacles and make turns to avoid collision.  

In order to autonomously drive a TurtleBot3 in the **TurtleBot3 world**, please follow the instruction below.

1. Terminate the turtlebot3\_teleop\_key node by entering `Ctrl` + `C` to the terminal that runs the teleop node.
2. Enter the below command to the terminal.

```
$ ros2 run turtlebot3_gazebo turtlebot3_drive

```

![](/assets/images/icon_unfold.png) Read more about **How to visualize Simulation data(RViz2)**

RViz2 visualizes published topics while simulation is running. You can launch RViz2 in a new terminal window by entering below command.

```
$ ros2 launch turtlebot3_bringup rviz2.launch.py

```

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_gazebo_rviz.png)

**NOTE**

* Please run the Simulation on **Remote PC**.
* Launching the Simulation for the first time on the Remote PC may take a while to setup the environment.

![](/assets/images/icon_unfold.png) Read more about **TurtleBot3 Simulation**

TurtleBot3 supports simulation development environment that can be programmed and developed with a virtual robot in the simulation. There are two development environments to do this, one is using a **fake node with 3D visualization tool RViz**, and the other is using the **3D robot simulator Gazebo**.

* The **fake node** is suitable for testing with the robot model and movement, but it does not support sensors.
* If you need to perform SLAM or Navigation, **Gazebo** would be a feasible solution as it supports sensors such as IMU, LDS, and camera.

In this instruction, Gazebo will be mainly introduced which is most widely used among ROS developers.

* **Gazebo Tutorials** : [http://gazebosim.org/tutorials](http://gazebosim.org/tutorials "http://gazebosim.org/tutorials")

## [Gazebo Simulation](#gazebo-simulation "#gazebo-simulation")

The contents in e-Manual can be updated without a prior notice and video contents could be outdated.

This Gazebo Simulation uses **ROS Gazebo package**, therefore, proper Gazebo version for ROS2 Foxy has to be installed before running this instruction.

### [Install Simulation Package](#install-simulation-package "#install-simulation-package")

The **TurtleBot3 Simulation Package** requires `turtlebot3` and `turtlebot3_msgs` packages as prerequisite. Without these prerequisite packages, the Simulation cannot be launched.  

Please follow the [PC Setup](/docs/en/platform/turtlebot3/quick-start/ "/docs/en/platform/turtlebot3/quick-start/") instructions if you did not install required packages and dependent packages.

```
$ cd ~/turtlebot3_ws/src/
$ git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/turtlebot3_ws && colcon build --symlink-install

```

### [Launch Simulation World](#launch-simulation-world "#launch-simulation-world")

Three simulation environments are prepared for TurtleBot3. Please select one of these environments to launch Gazebo.

**Please make sure to completely terminate other Simulation world before launching a new world.**

1. Empty World  

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_empty_world.png)

```
$ export TURTLEBOT3\_MODEL=burger
$ ros2 launch turtlebot3_gazebo empty_world.launch.py

```
2. TurtleBot3 World  

![](/assets/images/platform/turtlebot3/ros2/gazebo_world.png)

```
$ export TURTLEBOT3\_MODEL=waffle
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

```
3. TurtleBot3 House  

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_house.png)

```
$ export TURTLEBOT3\_MODEL=waffle_pi
$ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

```

**NOTE**: If TurtleBot3 House is launched for the first time, downloading the map may take more than a few minutes depending the network status.

### [Operate TurtleBot3](#operate-turtlebot3 "#operate-turtlebot3")

In order to teleoperate the TurtleBot3 with the keyboard, launch the teleoperation node with below command in a new terminal window.

```
$ ros2 run turtlebot3_teleop teleop_keyboard

```

![](/assets/images/icon_unfold.png) Read more about **How to run Autonomous Collision Avoidance**

A simple collision avoidance node is prepared which keeps certain distance from obstacles and make turns to avoid collision.  

In order to autonomously drive a TurtleBot3 in the **TurtleBot3 world**, please follow the instruction below.

1. Terminate the turtlebot3\_teleop\_key node by entering `Ctrl` + `C` to the terminal that runs the teleop node.
2. Enter the below command to the terminal.

```
$ ros2 run turtlebot3_gazebo turtlebot3_drive

```

![](/assets/images/icon_unfold.png) Read more about **How to visualize Simulation data(RViz2)**

RViz2 visualizes published topics while simulation is running. You can launch RViz2 in a new terminal window by entering below command.

```
$ ros2 launch turtlebot3_bringup rviz2.launch.py

```

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_gazebo_rviz.png)

**NOTE**

* Please run the Simulation on **Remote PC**.
* Launching the Simulation for the first time on the Remote PC may take a while to setup the environment.

![](/assets/images/icon_unfold.png) Read more about **TurtleBot3 Simulation**

TurtleBot3 supports simulation development environment that can be programmed and developed with a virtual robot in the simulation. There are two development environments to do this, one is using a **fake node with 3D visualization tool RViz**, and the other is using the **3D robot simulator Gazebo**.

* The **fake node** is suitable for testing with the robot model and movement, but it does not support sensors.
* If you need to perform SLAM or Navigation, **Gazebo** would be a feasible solution as it supports sensors such as IMU, LDS, and camera.

In this instruction, Gazebo will be mainly introduced which is most widely used among ROS developers.

* **Gazebo Tutorials** : [http://gazebosim.org/tutorials](http://gazebosim.org/tutorials "http://gazebosim.org/tutorials")

## [Gazebo Simulation](#gazebo-simulation "#gazebo-simulation")

The contents in e-Manual can be updated without a prior notice and video contents could be outdated.

This Gazebo Simulation uses **ROS Gazebo package**, therefore, proper Gazebo version for ROS2 Humble has to be installed before running this instruction.

### [Install Simulation Package](#install-simulation-package "#install-simulation-package")

The **TurtleBot3 Simulation Package** requires `turtlebot3` and `turtlebot3_msgs` packages as prerequisite. Without these prerequisite packages, the Simulation cannot be launched.  

Please follow the [PC Setup](/docs/en/platform/turtlebot3/quick-start/ "/docs/en/platform/turtlebot3/quick-start/") instructions if you did not install required packages and dependent packages.

```
$ cd ~/turtlebot3_ws/src/
$ git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/turtlebot3_ws && colcon build --symlink-install

```

### [Launch Simulation World](#launch-simulation-world "#launch-simulation-world")

Three simulation environments are prepared for TurtleBot3. Please select one of these environments to launch Gazebo.

**Please make sure to completely terminate other Simulation world before launching a new world.**

1. Empty World  

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_empty_world.png)

```
$ export TURTLEBOT3\_MODEL=burger
$ ros2 launch turtlebot3_gazebo empty_world.launch.py

```
2. TurtleBot3 World  

![](/assets/images/platform/turtlebot3/ros2/gazebo_world.png)

```
$ export TURTLEBOT3\_MODEL=waffle
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

```
3. TurtleBot3 House  

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_house.png)

```
$ export TURTLEBOT3\_MODEL=waffle_pi
$ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

```

**NOTE**: If TurtleBot3 House is launched for the first time, downloading the map may take more than a few minutes depending the network status.

### [Operate TurtleBot3](#operate-turtlebot3 "#operate-turtlebot3")

In order to teleoperate the TurtleBot3 with the keyboard, launch the teleoperation node with below command in a new terminal window.

```
$ ros2 run turtlebot3_teleop teleop_keyboard

```

![](/assets/images/icon_unfold.png) Read more about **How to run Autonomous Collision Avoidance**

A simple collision avoidance node is prepared which keeps certain distance from obstacles and make turns to avoid collision.  

In order to autonomously drive a TurtleBot3 in the **TurtleBot3 world**, please follow the instruction below.

1. Terminate the turtlebot3\_teleop\_key node by entering `Ctrl` + `C` to the terminal that runs the teleop node.
2. Enter the below command to the terminal.

```
$ ros2 run turtlebot3_gazebo turtlebot3_drive

```

![](/assets/images/icon_unfold.png) Read more about **How to visualize Simulation data(RViz2)**

RViz2 visualizes published topics while simulation is running. You can launch RViz2 in a new terminal window by entering below command.

```
$ ros2 launch turtlebot3_bringup rviz2.launch.py

```

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_gazebo_rviz.png)

**NOTE**

* Please run the Simulation on **Remote PC**.
* Launching the Simulation for the first time on the Remote PC may take a while to setup the environment.

![](/assets/images/icon_unfold.png) Read more about **TurtleBot3 Simulation**

TurtleBot3 supports simulation development environment that can be programmed and developed with a virtual robot in the simulation. There are two development environments to do this, one is using a **fake node with 3D visualization tool RViz**, and the other is using the **3D robot simulator Gazebo**.

* The **fake node** is suitable for testing with the robot model and movement, but it does not support sensors.
* If you need to perform SLAM or Navigation, **Gazebo** would be a feasible solution as it supports sensors such as IMU, LDS, and camera.

In this instruction, Gazebo will be mainly introduced which is most widely used among ROS developers.

* **Gazebo Tutorials** : [http://gazebosim.org/tutorials](http://gazebosim.org/tutorials "http://gazebosim.org/tutorials")

## [Gazebo Simulation](#gazebo-simulation "#gazebo-simulation")

The contents in e-Manual can be updated without a prior notice and video contents could be outdated.

This Gazebo Simulation uses **ROS Gazebo package**, therefore, proper Gazebo version must to be installed before running this instruction.

### [Launch Simulation](#launch-simulation "#launch-simulation")

If you are running ROS1 Noetic, please replace `melodic` with `noetic` in the command below.

1. To start the simulation, open one elevated command prompt.

```
> c:\opt\ros\melodic\x64\setup.bat
> c:\ws\turtlebot3\devel\setup.bat
> set TURTLEBOT3_MODEL=waffle
> roslaunch turtlebot3_fake turtlebot3_fake.launch

```
2. Then, open another elevated command prompt.

```
c:\opt\ros\melodic\x64\setup.bat
c:\ws\turtlebot3\devel\setup.bat
set TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_simulation.launch

```

 Previous Page
Next Page 
