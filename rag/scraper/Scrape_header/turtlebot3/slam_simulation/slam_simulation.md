
[Edit on GitHub](https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/simulation/slam_simulation.md "https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/simulation/slam_simulation.md") 

Kinetic 
Melodic
Noetic
Dashing
Foxy
Humble
Windows

## [SLAM Simulation](#slam-simulation "#slam-simulation")

When SLAM in Gazebo simulator, you can select or create various environments and robot models in virtual world. Other than preparing simulation environment instead of bringing up the robot, SLAM Simulation is pretty similar to that of [SLAM](/docs/en/platform/turtlebot3/slam/#slam "/docs/en/platform/turtlebot3/slam/#slam") with the actual TurtleBot3.

The following instructions require prerequisites from the previous sections, so please review to the [Simulation](/docs/en/platform/turtlebot3/simulation/ "/docs/en/platform/turtlebot3/simulation/") section first.

### Launch Simulation World

Three Gazebo environments are prepared, but for creating a map with SLAM, it is recommended to use either **TurtleBot3 World** or **TurtleBot3 House**.  

Use one of the following commands to load the Gazebo environment.

In this instruction, TurtleBot3 World will be used.  

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3\_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch

```

![](/assets/images/icon_unfold.png) Read more about **How to load TurtleBot3 House**

```
$ export TURTLEBOT3\_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch

```

### Run SLAM Node

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the SLAM node. Gmapping SLAM method is used by default.  

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3\_MODEL=burger
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

```

### Run Teleoperation Node

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the teleoperation node from the Remote PC.  

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3\_MODEL=burger
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

 Control Your TurtleBot3!
 ---------------------------
 Moving around:
        w
   a    s    d
        x

 w/x : increase/decrease linear velocity
 a/d : increase/decrease angular velocity
 space key, s : force stop

 CTRL-C to quit

```

### Save Map

When the map is created successfully, open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and save the map.

![](/assets/images/platform/turtlebot3/simulation/virtual_slam.png)

```
$ rosrun map_server map_saver -f ~/map

```

![](/assets/images/platform/turtlebot3/simulation/map.png)

> 
> The saved map.pgm file
> 
> 
> 

![](/assets/images/icon_unfold.png) Read more about **How to SLAM with multiple TurtleBot3**

In order to create a map with multiple robots, **multirobot-map-merge** package is required.  

Follow the instructions below instead of **Launching Simulation World** section of this page to operate multiple TurtleBot3.

1. Install necessary package

```
$ sudo apt-get install ros-kinetic-multirobot-map-merge

```
2. Load multiple TurtleBot3 in TurtleBot3 House.  

 These loaded turtlebot3s are set initial position and orientation.

```
$ roslaunch turtlebot3_gazebo multi_turtlebot3.launch

```

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_house_slam.png)
3. Launch SLAM for each TurtleBot3

```
$ ROS\_NAMESPACE=tb3_0 roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_0/base_footprint set_odom_frame:=tb3_0/odom set_map_frame:=tb3_0/map
$ ROS\_NAMESPACE=tb3_1 roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_1/base_footprint set_odom_frame:=tb3_1/odom set_map_frame:=tb3_1/map
$ ROS\_NAMESPACE=tb3_2 roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_2/base_footprint set_odom_frame:=tb3_2/odom set_map_frame:=tb3_2/map

```
4. Merge map data from each TurtleBot3

```
$ roslaunch turtlebot3_gazebo multi_map_merge.launch

```
5. Launch RViz

```
$ rosrun rviz rviz -d `rospack find turtlebot3_gazebo`/rviz/multi_turtlebot3_slam.rviz

```
6. Operate each TurtleBot3

```
$ ROS\_NAMESPACE=tb3_0 rosrun turtlebot3_teleop turtlebot3_teleop_key
$ ROS\_NAMESPACE=tb3_1 rosrun turtlebot3_teleop turtlebot3_teleop_key
$ ROS\_NAMESPACE=tb3_2 rosrun turtlebot3_teleop turtlebot3_teleop_key

```

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_house_slam1.png)
7. Save the Map

```
$ rosrun map_server map_saver -f ~/map

```

When SLAM in Gazebo simulator, you can select or create various environments and robot models in virtual world. Other than preparing simulation environment instead of bringing up the robot, SLAM Simulation is pretty similar to that of [SLAM](/docs/en/platform/turtlebot3/slam/#slam "/docs/en/platform/turtlebot3/slam/#slam") with the actual TurtleBot3.

The following instructions require prerequisites from the previous sections, so please review to the [Simulation](/docs/en/platform/turtlebot3/simulation/ "/docs/en/platform/turtlebot3/simulation/") section first.

### Launch Simulation World

Three Gazebo environments are prepared, but for creating a map with SLAM, it is recommended to use either **TurtleBot3 World** or **TurtleBot3 House**.  

Use one of the following commands to load the Gazebo environment.

In this instruction, TurtleBot3 World will be used.  

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3\_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch

```

![](/assets/images/icon_unfold.png) Read more about **How to load TurtleBot3 House**

```
$ export TURTLEBOT3\_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch

```

### Run SLAM Node

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the SLAM node. Gmapping SLAM method is used by default.  

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3\_MODEL=burger
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

```

### Run Teleoperation Node

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the teleoperation node from the Remote PC.  

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3\_MODEL=burger
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

 Control Your TurtleBot3!
 ---------------------------
 Moving around:
        w
   a    s    d
        x

 w/x : increase/decrease linear velocity
 a/d : increase/decrease angular velocity
 space key, s : force stop

 CTRL-C to quit

```

### Save Map

When the map is created successfully, open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and save the map.

![](/assets/images/platform/turtlebot3/simulation/virtual_slam.png)

```
$ rosrun map_server map_saver -f ~/map

```

![](/assets/images/platform/turtlebot3/simulation/map.png)

> 
> The saved map.pgm file
> 
> 
> 

![](/assets/images/icon_unfold.png) Read more about **How to SLAM with multiple TurtleBot3**

In order to create a map with multiple robots, **multirobot-map-merge** package is required.  

Follow the instructions below instead of **Launching Simulation World** section of this page to operate multiple TurtleBot3.

1. Install necessary package

```
$ sudo apt-get install ros-melodic-multirobot-map-merge

```
2. Load multiple TurtleBot3 in TurtleBot3 House.  

 These loaded turtlebot3s are set initial position and orientation.

```
$ roslaunch turtlebot3_gazebo multi_turtlebot3.launch

```

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_house_slam.png)
3. Launch SLAM for each TurtleBot3

```
$ ROS\_NAMESPACE=tb3_0 roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_0/base_footprint set_odom_frame:=tb3_0/odom set_map_frame:=tb3_0/map
$ ROS\_NAMESPACE=tb3_1 roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_1/base_footprint set_odom_frame:=tb3_1/odom set_map_frame:=tb3_1/map
$ ROS\_NAMESPACE=tb3_2 roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_2/base_footprint set_odom_frame:=tb3_2/odom set_map_frame:=tb3_2/map

```
4. Merge map data from each TurtleBot3

```
$ roslaunch turtlebot3_gazebo multi_map_merge.launch

```
5. Launch RViz

```
$ rosrun rviz rviz -d `rospack find turtlebot3_gazebo`/rviz/multi_turtlebot3_slam.rviz

```
6. Operate each TurtleBot3

```
$ ROS\_NAMESPACE=tb3_0 rosrun turtlebot3_teleop turtlebot3_teleop_key
$ ROS\_NAMESPACE=tb3_1 rosrun turtlebot3_teleop turtlebot3_teleop_key
$ ROS\_NAMESPACE=tb3_2 rosrun turtlebot3_teleop turtlebot3_teleop_key

```

![](/assets/images/platform/turtlebot3/simulation/turtlebot3_house_slam1.png)
7. Save the Map

```
$ rosrun map_server map_saver -f ~/map

```

When SLAM in Gazebo simulator, you can select or create various environments and robot models in virtual world. Other than preparing simulation environment instead of bringing up the robot, SLAM Simulation is pretty similar to that of [SLAM](/docs/en/platform/turtlebot3/slam/#slam "/docs/en/platform/turtlebot3/slam/#slam") with the actual TurtleBot3.

The following instructions require prerequisites from the previous sections, so please review to the [Simulation](/docs/en/platform/turtlebot3/simulation/ "/docs/en/platform/turtlebot3/simulation/") section first.

### Launch Simulation World

Three Gazebo environments are prepared, but for creating a map with SLAM, it is recommended to use either **TurtleBot3 World** or **TurtleBot3 House**.  

Use one of the following commands to load the Gazebo environment.

In this instruction, TurtleBot3 World will be used.  

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3\_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch

```

![](/assets/images/icon_unfold.png) Read more about **How to load TurtleBot3 House**

```
$ export TURTLEBOT3\_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch

```

### Run SLAM Node

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the SLAM node. Gmapping SLAM method is used by default.  

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3\_MODEL=burger
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

```

### Run Teleoperation Node

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the teleoperation node from the Remote PC.  

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3\_MODEL=burger
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

 Control Your TurtleBot3!
 ---------------------------
 Moving around:
        w
   a    s    d
        x

 w/x : increase/decrease linear velocity
 a/d : increase/decrease angular velocity
 space key, s : force stop

 CTRL-C to quit

```

### Save Map

When the map is created successfully, open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and save the map.

![](/assets/images/platform/turtlebot3/simulation/virtual_slam.png)

```
$ rosrun map_server map_saver -f ~/map

```

![](/assets/images/platform/turtlebot3/simulation/map.png)

> 
> The saved map.pgm file
> 
> 
> 

When SLAM in Gazebo simulator, you can select or create various environments and robot models in virtual world. Other than preparing simulation environment instead of bringing up the robot, SLAM Simulation is pretty similar to that of [SLAM](/docs/en/platform/turtlebot3/slam/#slam "/docs/en/platform/turtlebot3/slam/#slam") with the actual TurtleBot3.

The following instructions require prerequisites from the previous sections, so please review to the [Simulation](/docs/en/platform/turtlebot3/simulation/ "/docs/en/platform/turtlebot3/simulation/") section first.

### Launch Simulation World

Three Gazebo environments are prepared, but for creating a map with SLAM, it is recommended to use either **TurtleBot3 World** or **TurtleBot3 House**.  

Use one of the following commands to load the Gazebo environment.

In this instruction, TurtleBot3 World will be used.  

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3\_MODEL=burger
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

```

![](/assets/images/icon_unfold.png) Read more about **How to load TurtleBot3 House**

```
$ export TURTLEBOT3\_MODEL=burger
$ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

```

### Run SLAM Node

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the SLAM node. Cartographer SLAM method is used by default.

```
$ export TURTLEBOT3\_MODEL=burger
$ ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

```

### Run Teleoperation Node

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the teleoperation node from the Remote PC.

```
$ export TURTLEBOT3\_MODEL=burger
$ ros2 run turtlebot3_teleop teleop_keyboard

 Control Your TurtleBot3!
 ---------------------------
 Moving around:
        w
   a    s    d
        x

 w/x : increase/decrease linear velocity
 a/d : increase/decrease angular velocity
 space key, s : force stop

 CTRL-C to quit

```

### Save Map

When the map is created successfully, open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and save the map.

![](/assets/images/platform/turtlebot3/simulation/virtual_slam.png)

```
$ ros2 run nav2_map_server map_saver -f ~/map

```

![](/assets/images/platform/turtlebot3/simulation/map.png)

> 
> The saved map.pgm file
> 
> 
> 

When SLAM in Gazebo simulator, you can select or create various environments and robot models in virtual world. Other than preparing simulation environment instead of bringing up the robot, SLAM Simulation is pretty similar to that of [SLAM](/docs/en/platform/turtlebot3/slam/#slam "/docs/en/platform/turtlebot3/slam/#slam") with the actual TurtleBot3.

The following instructions require prerequisites from the previous sections, so please review to the [Simulation](/docs/en/platform/turtlebot3/simulation/ "/docs/en/platform/turtlebot3/simulation/") section first.

### Launch Simulation World

Three Gazebo environments are prepared, but for creating a map with SLAM, it is recommended to use either **TurtleBot3 World** or **TurtleBot3 House**.  

Use one of the following commands to load the Gazebo environment.

In this instruction, TurtleBot3 World will be used.  

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3\_MODEL=burger
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

```

![](/assets/images/icon_unfold.png) Read more about **How to load TurtleBot3 House**

```
$ export TURTLEBOT3\_MODEL=burger
$ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

```

### Run SLAM Node

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the SLAM node. Cartographer SLAM method is used by default.

```
$ export TURTLEBOT3\_MODEL=burger
$ ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

```

### Run Teleoperation Node

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the teleoperation node from the Remote PC.

```
$ export TURTLEBOT3\_MODEL=burger
$ ros2 run turtlebot3_teleop teleop_keyboard

 Control Your TurtleBot3!
 ---------------------------
 Moving around:
        w
   a    s    d
        x

 w/x : increase/decrease linear velocity
 a/d : increase/decrease angular velocity
 space key, s : force stop

 CTRL-C to quit

```

### Save Map

When the map is created successfully, open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and save the map.

![](/assets/images/platform/turtlebot3/simulation/virtual_slam.png)

```
$ ros2 run nav2_map_server map_saver_cli -f ~/map

```

![](/assets/images/platform/turtlebot3/simulation/map.png)

> 
> The saved map.pgm file
> 
> 
> 

When SLAM in Gazebo simulator, you can select or create various environments and robot models in virtual world. Other than preparing simulation environment instead of bringing up the robot, SLAM Simulation is pretty similar to that of [SLAM](/docs/en/platform/turtlebot3/slam/#slam "/docs/en/platform/turtlebot3/slam/#slam") with the actual TurtleBot3.

The following instructions require prerequisites from the previous sections, so please review to the [Simulation](/docs/en/platform/turtlebot3/simulation/ "/docs/en/platform/turtlebot3/simulation/") section first.

### Launch Simulation World

Three Gazebo environments are prepared, but for creating a map with SLAM, it is recommended to use either **TurtleBot3 World** or **TurtleBot3 House**.  

Use one of the following commands to load the Gazebo environment.

In this instruction, TurtleBot3 World will be used.  

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
$ export TURTLEBOT3\_MODEL=burger
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

```

![](/assets/images/icon_unfold.png) Read more about **How to load TurtleBot3 House**

```
$ export TURTLEBOT3\_MODEL=burger
$ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

```

### Run SLAM Node

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the SLAM node. Cartographer SLAM method is used by default.

```
$ export TURTLEBOT3\_MODEL=burger
$ ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

```

### Run Teleoperation Node

Open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and run the teleoperation node from the Remote PC.

```
$ export TURTLEBOT3\_MODEL=burger
$ ros2 run turtlebot3_teleop teleop_keyboard

 Control Your TurtleBot3!
 ---------------------------
 Moving around:
        w
   a    s    d
        x

 w/x : increase/decrease linear velocity
 a/d : increase/decrease angular velocity
 space key, s : force stop

 CTRL-C to quit

```

### Save Map

When the map is created successfully, open a new terminal from Remote PC with `Ctrl` + `Alt` + `T` and save the map.

![](/assets/images/platform/turtlebot3/simulation/virtual_slam.png)

```
$ ros2 run nav2_map_server map_saver_cli -f ~/map

```

![](/assets/images/platform/turtlebot3/simulation/map.png)

> 
> The saved map.pgm file
> 
> 
> 

When SLAM in Gazebo simulator, you can select or create various environments and robot models in virtual world. Other than preparing simulation environment instead of bringing up the robot, SLAM Simulation is pretty similar to that of [SLAM](/docs/en/platform/turtlebot3/slam/#slam "/docs/en/platform/turtlebot3/slam/#slam") with the actual TurtleBot3.

The following instructions require prerequisites from the previous sections, so please review to the [Simulation](/docs/en/platform/turtlebot3/simulation/ "/docs/en/platform/turtlebot3/simulation/") section first.

### Launch SLAM Simulation

If you are running ROS1 Noetic, please replace `melodic` with `noetic` in the command below.

Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
> c:\opt\ros\melodic\x64\setup.bat
> c:\ws\turtlebot3\devel\setup.bat
> set TURTLEBOT3_MODEL=waffle

> curl -o turtlebot3_demo.launch https://raw.githubusercontent.com/ms-iot/ROSOnWindows/master/docs/Turtlebot/turtlebot3_demo.launch
> roslaunch turtlebot3_demo.launch

```

After a few moments, you will see Gazebo running a simulated world with your simulated TurtleBot3, RViz running the mapping progress, and a simulation node to drive the TurtleBot3 random walking.

![](https://ms-iot.github.io/ROSOnWindows/Extras/Turtlebot3_Gazebo_SLAM.gif)

 Previous Page
Next Page 
