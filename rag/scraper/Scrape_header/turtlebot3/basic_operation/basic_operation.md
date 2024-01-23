
[Edit on GitHub](https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/quick_start/basic_operation.md "https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/quick_start/basic_operation.md") 

Kinetic 
Melodic
Noetic
Dashing
Foxy
Humble
Windows

## [Basic Operation](#basic-operation "#basic-operation")

### [Teleoperation](#teleoperation "#teleoperation")

**WARNING**: Make sure to run the [Bringup](/docs/en/platform/turtlebot3/bringup/#bringup "/docs/en/platform/turtlebot3/bringup/#bringup") from the TurtleBot3 SBC before teleoperation. Teleoperate the robot, and be careful when testing the robot on the table as the robot might fall.

The TurtleBot3 can be teleoperated by various remote controllers. Make sure that the necessary ROS packages are supported for your SBC and ROS version.

#### [Keyboard](#keyboard "#keyboard")

**TIP**: Before executing this command, you have to specify the model name of TurtleBot3. The `${TB3_MODEL}` is the name of the model you are using in `burger`, `waffle`, `waffle_pi`.

1. Launch `turtlebot3_teleop_key` node from Remote PC for the teleoperation using a keyboard. Replace the `${TB3_MODEL}` parameter with your model name such as `burger`, `waffle`, `waffle_pi`.

```
$ export TURTLEBOT3\_MODEL=${TB3\_MODEL}
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

```
2. If the node is successfully launched, the following instruction will be appeared to the terminal window.

```
Control Your Turtlebot3
Moving around
     w
 a   s   d
     x
w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop
CTRL-C to quit

```

![](/assets/images/icon_unfold.png) **Read more about How to predefine the TURTLEBOT3\_MODEL**

The `export TURTLEBOT3_MODEL=${TB3_MODEL}` command can be omitted if the **TURTLEBOT3\_MODEL** parameter is predefined in the `.bashrc` file. The `.bashrc` file is automatically loaded when a terminal window is created.

* Example of defining TurtlBot3 Burger as a default.

```
$ echo 'export TURTLEBOT3\_MODEL=burger' >> ~/.bashrc
$ source ~/.bashrc

```
* Example of defining TurtlBot3 Waffle Pi as a default.

```
$ echo 'export TURTLEBOT3\_MODEL=waffle\_pi' >> ~/.bashrc
$ source ~/.bashrc

```

#### [RC100](#rc100 "#rc100")

The settings for [ROBOTIS RC-100B](/docs/en/parts/communication/rc-100/ "/docs/en/parts/communication/rc-100/") controller is included in the OpenCR firmware for TurtleBot3 Burger, Waffle and Waffle Pi. This controller can be used with the Bluetooth module [BT410](/docs/en/parts/communication/bt-410/ "/docs/en/parts/communication/bt-410/"). The TurtleBot3 Waffle Pi includes the RC-100 controller and Bluetooth modules. When using RC-100, it is not necessary to execute a specific node because `turtlebot_core` node creates a `/cmd_vel` topic in the firmware directly connected to OpenCR.

![](/assets/images/platform/turtlebot3/example/rc100b_with_bt410.png)

1. Connect BT-410 to OpenCR UART1 port (as described [here](/docs/en/platform/turtlebot3/appendix_opencr1_0/ "/docs/en/platform/turtlebot3/appendix_opencr1_0/")).
2. Control TurtleBot3 with RC-100.

	* Up / Down : Increase or decrease linear velocity
	* Left / Right : Increase or decrease angular velocity

#### [PS3 Joystick](#ps3-joystick "#ps3-joystick")

1. Connect the PS3 Joystick to the remote PC via Bluetooth or a USB cable.
2. Install packages for teleoperation using PS3 joystick.

```
$ sudo apt-get install ros-kinetic-joy ros-kinetic-joystick-drivers ros-kinetic-teleop-twist-joy

```
3. Launch the teleoperation node.

```
$ roslaunch teleop_twist_joy teleop.launch

```

#### [XBOX 360 Joystick](#xbox-360-joystick "#xbox-360-joystick")

1. Connect XBOX 360 Joystick to the remote PC with Wireless Adapter or USB cable.
2. Install packages for teleoperation using XBOX 360 joystick.

```
$ sudo apt-get install xboxdrv ros-kinetic-joy ros-kinetic-joystick-drivers ros-kinetic-teleop-twist-joy

```
3. Launch teleoperation packages for XBOX 360 joystick.

```
$ sudo xboxdrv --silent
$ roslaunch teleop_twist_joy teleop.launch

```

#### [Wii Remote](#wii-remote "#wii-remote")

1. Connect Wii remote to the remote PC via Bluetooth.
2. Install packages for teleoperation using Wii remote.

```
$ sudo apt-get install ros-kinetic-wiimote libbluetooth-dev libcwiid-dev
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-drivers/joystick_drivers.git  
$ cd ~/catkin_ws && catkin_make

```
3. Run teleoperation packages for Wii remote.

```
$ rosrun wiimote wiimote_node
$ rosrun wiimote teleop_wiimote

```

### [Topic Monitor](#topic-monitor "#topic-monitor")

![](/assets/images/icon_unfold.png) **Read more about Topic Monitor**

In order to check the topics of TurtleBot3, we will use [rqt](http://wiki.ros.org/rqt "http://wiki.ros.org/rqt") provided by ROS. The rqt is a Qt-based framework for GUI development for ROS. The rqt is a tool that allows users to easily see the topic status by displaying all the topics in the topic list. There are topic names, types, bandwidth, Hz, value in GUI.

1. Run the rqt from PC with the command below. If the topic monitor window is not displayed, select the `plugin` -> `Topics` -> `Topic Monitor`.

```
$ rqt

```

![](/assets/images/platform/turtlebot3/example/rqt_1.png)
2. When the topic monitor loaded, the topic values are not monitored. Click the checkbox next to each topic to monitor the topic.  

![](/assets/images/platform/turtlebot3/example/rqt_2.png)
3. To see more detailed topic message, click the `▶` icon next to the checkbox.  

![](/assets/images/platform/turtlebot3/example/rqt_3.png)

* `/battery_state` indicates a message relating to the battery condition, such as the current battery voltage and remaining capacity.  

![](/assets/images/platform/turtlebot3/example/rqt_4.png)
* `/diagnostics` indicates a message the status of the components connected to the TurtleBot3, such as a MPU9250, DYNAMIXEL-X, a HLS-LFCD-LDS, a battery and a OpenCR.  

![](/assets/images/platform/turtlebot3/example/rqt_5.png)
* `/odom` indicates a message the odometry of the TurtleBot3. This topic has orientation and position by the encoder data.  

![](/assets/images/platform/turtlebot3/example/rqt_6.png)
* `/sensor_state` indicates a message the encoder values, battery and torque.  

![](/assets/images/platform/turtlebot3/example/rqt_7.png)
* `/scan` indicates a message all of the LDS data, such as angle\_max and min, range\_max and min, indicates, ranges.  

![](/assets/images/platform/turtlebot3/example/rqt_8.png)

## [Basic Operation](#basic-operation "#basic-operation")

### [Teleoperation](#teleoperation "#teleoperation")

**WARNING**: Make sure to run the [Bringup](/docs/en/platform/turtlebot3/bringup/#bringup "/docs/en/platform/turtlebot3/bringup/#bringup") from the TurtleBot3 SBC before teleoperation. Teleoperate the robot, and be careful when testing the robot on the table as the robot might fall.

The TurtleBot3 can be teleoperated by various remote controllers. Make sure that the necessary ROS packages are supported for your SBC and ROS version.

#### [Keyboard](#keyboard "#keyboard")

**TIP**: Before executing this command, you have to specify the model name of TurtleBot3. The `${TB3_MODEL}` is the name of the model you are using in `burger`, `waffle`, `waffle_pi`.

1. Launch `turtlebot3_teleop_key` node from Remote PC for the teleoperation using a keyboard. Replace the `${TB3_MODEL}` parameter with your model name such as `burger`, `waffle`, `waffle_pi`.

```
$ export TURTLEBOT3\_MODEL=${TB3\_MODEL}
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

```
2. If the node is successfully launched, the following instruction will be appeared to the terminal window.

```
Control Your Turtlebot3
Moving around
     w
 a   s   d
     x
w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop
CTRL-C to quit

```

![](/assets/images/icon_unfold.png) **Read more about How to predefine the TURTLEBOT3\_MODEL**

The `export TURTLEBOT3_MODEL=${TB3_MODEL}` command can be omitted if the **TURTLEBOT3\_MODEL** parameter is predefined in the `.bashrc` file. The `.bashrc` file is automatically loaded when a terminal window is created.

* Example of defining TurtlBot3 Burger as a default.

```
$ echo 'export TURTLEBOT3\_MODEL=burger' >> ~/.bashrc
$ source ~/.bashrc

```
* Example of defining TurtlBot3 Waffle Pi as a default.

```
$ echo 'export TURTLEBOT3\_MODEL=waffle\_pi' >> ~/.bashrc
$ source ~/.bashrc

```

#### [RC100](#rc100 "#rc100")

The settings for [ROBOTIS RC-100B](/docs/en/parts/communication/rc-100/ "/docs/en/parts/communication/rc-100/") controller is included in the OpenCR firmware for TurtleBot3 Burger, Waffle and Waffle Pi. This controller can be used with the Bluetooth module [BT410](/docs/en/parts/communication/bt-410/ "/docs/en/parts/communication/bt-410/"). The TurtleBot3 Waffle Pi includes the RC-100 controller and Bluetooth modules. When using RC-100, it is not necessary to execute a specific node because `turtlebot_core` node creates a `/cmd_vel` topic in the firmware directly connected to OpenCR.

![](/assets/images/platform/turtlebot3/example/rc100b_with_bt410.png)

1. Connect BT-410 to OpenCR UART1 port (as described [here](/docs/en/platform/turtlebot3/appendix_opencr1_0/ "/docs/en/platform/turtlebot3/appendix_opencr1_0/")).
2. Control TurtleBot3 with RC-100.

	* Up / Down : Increase or decrease linear velocity
	* Left / Right : Increase or decrease angular velocity

#### [PS3 Joystick](#ps3-joystick "#ps3-joystick")

1. Connect the PS3 Joystick to the remote PC via Bluetooth or a USB cable.
2. Install packages for teleoperation using PS3 joystick.

```
$ sudo apt-get install ros-melodic-joy ros-melodic-joystick-drivers ros-melodic-teleop-twist-joy

```
3. Launch the teleoperation node.

```
$ roslaunch teleop_twist_joy teleop.launch

```

#### [XBOX 360 Joystick](#xbox-360-joystick "#xbox-360-joystick")

1. Connect XBOX 360 Joystick to the remote PC with Wireless Adapter or USB cable.
2. Install packages for teleoperation using XBOX 360 joystick.

```
$ sudo apt-get install xboxdrv ros-melodic-joy ros-melodic-joystick-drivers ros-melodic-teleop-twist-joy

```
3. Launch teleoperation packages for XBOX 360 joystick.

```
$ sudo xboxdrv --silent
$ roslaunch teleop_twist_joy teleop.launch

```

#### [Wii Remote](#wii-remote "#wii-remote")

1. Connect Wii remote to the remote PC via Bluetooth.
2. Install packages for teleoperation using Wii remote.

```
$ sudo apt-get install ros-melodic-wiimote libbluetooth-dev libcwiid-dev
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-drivers/joystick_drivers.git  
$ cd ~/catkin_ws && catkin_make

```
3. Run teleoperation packages for Wii remote.

```
$ rosrun wiimote wiimote_node
$ rosrun wiimote teleop_wiimote

```

### [Topic Monitor](#topic-monitor "#topic-monitor")

![](/assets/images/icon_unfold.png) **Read more about Topic Monitor**

In order to check the topics of TurtleBot3, we will use [rqt](http://wiki.ros.org/rqt "http://wiki.ros.org/rqt") provided by ROS. The rqt is a Qt-based framework for GUI development for ROS. The rqt is a tool that allows users to easily see the topic status by displaying all the topics in the topic list. There are topic names, types, bandwidth, Hz, value in GUI.

1. Run the rqt from PC with the command below. If the topic monitor window is not displayed, select the `plugin` -> `Topics` -> `Topic Monitor`.

```
$ rqt

```

![](/assets/images/platform/turtlebot3/example/rqt_1.png)
2. When the topic monitor loaded, the topic values are not monitored. Click the checkbox next to each topic to monitor the topic.  

![](/assets/images/platform/turtlebot3/example/rqt_2.png)
3. To see more detailed topic message, click the `▶` icon next to the checkbox.  

![](/assets/images/platform/turtlebot3/example/rqt_3.png)

* `/battery_state` indicates a message relating to the battery condition, such as the current battery voltage and remaining capacity.  

![](/assets/images/platform/turtlebot3/example/rqt_4.png)
* `/diagnostics` indicates a message the status of the components connected to the TurtleBot3, such as a MPU9250, DYNAMIXEL-X, a HLS-LFCD-LDS, a battery and a OpenCR.  

![](/assets/images/platform/turtlebot3/example/rqt_5.png)
* `/odom` indicates a message the odometry of the TurtleBot3. This topic has orientation and position by the encoder data.  

![](/assets/images/platform/turtlebot3/example/rqt_6.png)
* `/sensor_state` indicates a message the encoder values, battery and torque.  

![](/assets/images/platform/turtlebot3/example/rqt_7.png)
* `/scan` indicates a message all of the LDS data, such as angle\_max and min, range\_max and min, indicates, ranges.  

![](/assets/images/platform/turtlebot3/example/rqt_8.png)

## [Basic Operation](#basic-operation "#basic-operation")

### [Teleoperation](#teleoperation "#teleoperation")

**WARNING**: Make sure to run the [Bringup](/docs/en/platform/turtlebot3/bringup/#bringup "/docs/en/platform/turtlebot3/bringup/#bringup") from the TurtleBot3 SBC before teleoperation. Teleoperate the robot, and be careful when testing the robot on the table as the robot might fall.

The TurtleBot3 can be teleoperated by various remote controllers. Make sure that the necessary ROS packages are supported for your SBC and ROS version.

#### [Keyboard](#keyboard "#keyboard")

**TIP**: Before executing this command, you have to specify the model name of TurtleBot3. The `${TB3_MODEL}` is the name of the model you are using in `burger`, `waffle`, `waffle_pi`.

1. Launch `turtlebot3_teleop_key` node from Remote PC for the teleoperation using a keyboard. Replace the `${TB3_MODEL}` parameter with your model name such as `burger`, `waffle`, `waffle_pi`.

```
$ export TURTLEBOT3\_MODEL=${TB3\_MODEL}
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

```
2. If the node is successfully launched, the following instruction will be appeared to the terminal window.

```
Control Your Turtlebot3
Moving around
     w
 a   s   d
     x
w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop
CTRL-C to quit

```

![](/assets/images/icon_unfold.png) **Read more about How to predefine the TURTLEBOT3\_MODEL**

The `export TURTLEBOT3_MODEL=${TB3_MODEL}` command can be omitted if the **TURTLEBOT3\_MODEL** parameter is predefined in the `.bashrc` file. The `.bashrc` file is automatically loaded when a terminal window is created.

* Example of defining TurtlBot3 Burger as a default.

```
$ echo 'export TURTLEBOT3\_MODEL=burger' >> ~/.bashrc
$ source ~/.bashrc

```
* Example of defining TurtlBot3 Waffle Pi as a default.

```
$ echo 'export TURTLEBOT3\_MODEL=waffle\_pi' >> ~/.bashrc
$ source ~/.bashrc

```

#### [RC100](#rc100 "#rc100")

The settings for [ROBOTIS RC-100B](/docs/en/parts/communication/rc-100/ "/docs/en/parts/communication/rc-100/") controller is included in the OpenCR firmware for TurtleBot3 Burger, Waffle and Waffle Pi. This controller can be used with the Bluetooth module [BT410](/docs/en/parts/communication/bt-410/ "/docs/en/parts/communication/bt-410/"). The TurtleBot3 Waffle Pi includes the RC-100 controller and Bluetooth modules. When using RC-100, it is not necessary to execute a specific node because `turtlebot_core` node creates a `/cmd_vel` topic in the firmware directly connected to OpenCR.

![](/assets/images/platform/turtlebot3/example/rc100b_with_bt410.png)

1. Connect BT-410 to OpenCR UART1 port (as described [here](/docs/en/platform/turtlebot3/appendix_opencr1_0/ "/docs/en/platform/turtlebot3/appendix_opencr1_0/")).
2. Control TurtleBot3 with RC-100.

	* Up / Down : Increase or decrease linear velocity
	* Left / Right : Increase or decrease angular velocity

#### [PS3 Joystick](#ps3-joystick "#ps3-joystick")

1. Connect the PS3 Joystick to the remote PC via Bluetooth or a USB cable.
2. Install packages for teleoperation using PS3 joystick.

```
$ sudo apt install ros-noetic-joy ros-noetic-joystick-drivers ros-noetic-teleop-twist-joy

```
3. Launch the teleoperation node.

```
$ roslaunch teleop_twist_joy teleop.launch

```

#### [XBOX 360 Joystick](#xbox-360-joystick "#xbox-360-joystick")

1. Connect XBOX 360 Joystick to the remote PC with Wireless Adapter or USB cable.
2. Install packages for teleoperation using XBOX 360 joystick.

```
$ sudo apt install xboxdrv ros-noetic-joy ros-noetic-joystick-drivers ros-noetic-teleop-twist-joy

```
3. Launch teleoperation packages for XBOX 360 joystick.

```
$ sudo xboxdrv --silent
$ roslaunch teleop_twist_joy teleop.launch

```

#### [Wii Remote](#wii-remote "#wii-remote")

1. Connect Wii remote to the remote PC via Bluetooth.
2. Install packages for teleoperation using Wii remote.

```
$ sudo apt install ros-noetic-wiimote libbluetooth-dev libcwiid-dev
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-drivers/joystick_drivers.git  
$ cd ~/catkin_ws && catkin_make

```
3. Run teleoperation packages for Wii remote.

```
$ rosrun wiimote wiimote_node
$ rosrun wiimote teleop_wiimote

```

### [Topic Monitor](#topic-monitor "#topic-monitor")

![](/assets/images/icon_unfold.png) **Read more about Topic Monitor**

In order to check the topics of TurtleBot3, we will use [rqt](http://wiki.ros.org/rqt "http://wiki.ros.org/rqt") provided by ROS. The rqt is a Qt-based framework for GUI development for ROS. The rqt is a tool that allows users to easily see the topic status by displaying all the topics in the topic list. There are topic names, types, bandwidth, Hz, value in GUI.

1. Run the rqt from PC with the command below. If the topic monitor window is not displayed, select the `plugin` -> `Topics` -> `Topic Monitor`.

```
$ rqt

```

![](/assets/images/platform/turtlebot3/example/rqt_1.png)
2. When the topic monitor loaded, the topic values are not monitored. Click the checkbox next to each topic to monitor the topic.  

![](/assets/images/platform/turtlebot3/example/rqt_2.png)
3. To see more detailed topic message, click the `▶` icon next to the checkbox.  

![](/assets/images/platform/turtlebot3/example/rqt_3.png)

* `/battery_state` indicates a message relating to the battery condition, such as the current battery voltage and remaining capacity.  

![](/assets/images/platform/turtlebot3/example/rqt_4.png)
* `/diagnostics` indicates a message the status of the components connected to the TurtleBot3, such as a MPU9250, DYNAMIXEL-X, a HLS-LFCD-LDS, a battery and a OpenCR.  

![](/assets/images/platform/turtlebot3/example/rqt_5.png)
* `/odom` indicates a message the odometry of the TurtleBot3. This topic has orientation and position by the encoder data.  

![](/assets/images/platform/turtlebot3/example/rqt_6.png)
* `/sensor_state` indicates a message the encoder values, battery and torque.  

![](/assets/images/platform/turtlebot3/example/rqt_7.png)
* `/scan` indicates a message all of the LDS data, such as angle\_max and min, range\_max and min, indicates, ranges.  

![](/assets/images/platform/turtlebot3/example/rqt_8.png)

## [Basic Operation](#basic-operation "#basic-operation")

### [Teleoperation](#teleoperation "#teleoperation")

**WARNING**: Make sure to run the [Bringup](/docs/en/platform/turtlebot3/bringup/#bringup "/docs/en/platform/turtlebot3/bringup/#bringup") from the TurtleBot3 SBC before teleoperation. Teleoperate the robot, and be careful when testing the robot on the table as the robot might fall.

The TurtleBot3 can be teleoperated by various remote controllers. Make sure that the necessary ROS packages are supported for your SBC and ROS version.

#### [Keyboard](#keyboard "#keyboard")

1. Open a terminal on **Remote PC**.
2. Run teleoperation node. Replace the `${TB3_MODEL}` with `burger` or `waffle` or `waffle_pi`, if TURTLEBOT3\_MODEL parameter is not predefined.

```
$ export TURTLEBOT3\_MODEL=${TB3\_MODEL}
$ ros2 run turtlebot3_teleop teleop_keyboard

```
3. If the node is successfully launched, the following instruction will be appeared to the terminal window.

```
Control Your Turtlebot3
Moving around
     w
 a   s   d
     x
w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)
space key, s : force stop
CTRL-C to quit

```

#### [RC-100](#rc100 "#rc100")

The settings for [ROBOTIS RC-100B](/docs/en/parts/communication/rc-100/ "/docs/en/parts/communication/rc-100/") is included in an OpenCR firmware for TurtleBot3. It can be used with the Bluetooth module [BT410](/docs/en/parts/communication/bt-410/ "/docs/en/parts/communication/bt-410/"). TurtleBot3 Waffle Pi includes this controller and Bluetooth modules. When using RC-100B, it is not necessary to execute a specific node because `turtlebot_core` node creates a `/cmd_vel` topic in the firmware directly connected to OpenCR.

![](/assets/images/platform/turtlebot3/example/rc100b_with_bt410.png)

1. Connect BT-410 to OpenCR UART1 port (as described [here](/docs/en/platform/turtlebot3/appendix_opencr1_0/ "/docs/en/platform/turtlebot3/appendix_opencr1_0/")).
2. Control TurtleBot3 with RC-100.

	* Up / Down : Increase or decrease linear velocity
	* Left / Right : Increase or decrease angular velocity

#### [PS3 Joystick](#ps3-joystick "#ps3-joystick")

1. Connect PS3 Joystick to **Remote PC** via Bluetooth or a USB cable.
2. Install `ds4drv` packages by using pip.

```
$ sudo pip install ds4drv

```
3. Launch the joystick node.

```
$ sudo ds4drv
$ ros2 run joy joy_node

```
4. Open a new terminal and launch the teleoperation node.

```
$ ros2 run teleop_twist_joy teleop_node

```

#### [XBOX 360 Joystick](#xbox-360-joystick "#xbox-360-joystick")

1. Connect XBOX 360 Joystick to the remote PC with Wireless Adapter or USB cable.
2. Open a terminal on **Remote PC**.
3. Launch the joystick node.

```
$ ros2 run joy joy_node

```
4. Open a new terminal and launch the teleoperation node.

```
$ ros2 run teleop_twist_joy teleop_node

```

### [Topic Monitor](#topic-monitor "#topic-monitor")

![](/assets/images/icon_unfold.png) **Read more about Topic Monitor**

In order to check topics of TurtleBot3, Use [rqt](http://wiki.ros.org/rqt "http://wiki.ros.org/rqt") provided by ROS, which is a Qt-based framework for GUI development for ROS. It is a tool displaying all topics of TurtleBot3 with a topic name, type, bandwidth, Hz, and value.

1. Run the rqt from PC with the command below. If the topic monitor window is not displayed, select the `plugin` -> `Topics` -> `Topic Monitor`.

```
$ rqt

```

![](/assets/images/platform/turtlebot3/ros2/rqt_1.png)
2. When the topic monitor loaded, the topic values are not monitored. Click the checkbox next to each topic to monitor the topic.  

![](/assets/images/platform/turtlebot3/ros2/rqt_2.png)
3. To see more detailed topic message, click the `▶` icon next to the checkbox.  

![](/assets/images/platform/turtlebot3/ros2/rqt_3.png)

* `/battery_state` indicates a message relating to the battery condition, such as the current battery voltage and remaining capacity.  

![](/assets/images/platform/turtlebot3/ros2/rqt_4.png)
* `/odom` indicates a message the odometry of the TurtleBot3. This topic has orientation and position by the encoder data.  

![](/assets/images/platform/turtlebot3/ros2/rqt_5.png)
* `/sensor_state` indicates a message the encoder values, battery and torque.  

![](/assets/images/platform/turtlebot3/ros2/rqt_6.png)
* `/scan` indicates a message all of the LDS data, such as angle\_max and min, range\_max and min, indicates, ranges.  

![](/assets/images/platform/turtlebot3/ros2/rqt_7.png)

## [Basic Operation](#basic-operation "#basic-operation")

### [Teleoperation](#teleoperation "#teleoperation")

**WARNING**: Make sure to run the [Bringup](/docs/en/platform/turtlebot3/bringup/#bringup "/docs/en/platform/turtlebot3/bringup/#bringup") from the TurtleBot3 SBC before teleoperation. Teleoperate the robot, and be careful when testing the robot on the table as the robot might fall.

The TurtleBot3 can be teleoperated by various remote controllers. Make sure that the necessary ROS packages are supported for your SBC and ROS version.

#### [Keyboard](#keyboard "#keyboard")

1. Open a terminal on **Remote PC**.
2. Run teleoperation node. Replace the `${TB3_MODEL}` with `burger` or `waffle` or `waffle_pi`, if TURTLEBOT3\_MODEL parameter is not predefined.

```
$ export TURTLEBOT3\_MODEL=${TB3\_MODEL}
$ ros2 run turtlebot3_teleop teleop_keyboard

```
3. If the node is successfully launched, the following instruction will be appeared to the terminal window.

```
Control Your Turtlebot3
Moving around
     w
 a   s   d
     x
w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)
space key, s : force stop
CTRL-C to quit

```

#### [RC-100](#rc100 "#rc100")

The settings for [ROBOTIS RC-100B](/docs/en/parts/communication/rc-100/ "/docs/en/parts/communication/rc-100/") is included in an OpenCR firmware for TurtleBot3. It can be used with the Bluetooth module [BT410](/docs/en/parts/communication/bt-410/ "/docs/en/parts/communication/bt-410/"). TurtleBot3 Waffle Pi includes this controller and Bluetooth modules. When using RC-100B, it is not necessary to execute a specific node because `turtlebot_core` node creates a `/cmd_vel` topic in the firmware directly connected to OpenCR.

![](/assets/images/platform/turtlebot3/example/rc100b_with_bt410.png)

1. Connect BT-410 to OpenCR UART1 port (as described [here](/docs/en/platform/turtlebot3/appendix_opencr1_0/ "/docs/en/platform/turtlebot3/appendix_opencr1_0/")).
2. Control TurtleBot3 with RC-100.

	* Up / Down : Increase or decrease linear velocity
	* Left / Right : Increase or decrease angular velocity

#### [PS3 Joystick](#ps3-joystick "#ps3-joystick")

1. Connect PS3 Joystick to **Remote PC** via Bluetooth or a USB cable.
2. Install `ds4drv` packages by using pip.

```
$ sudo pip install ds4drv

```
3. Launch the joystick node.

```
$ sudo ds4drv
$ ros2 run joy joy_node

```
4. Open a new terminal and launch the teleoperation node.

```
$ ros2 run teleop_twist_joy teleop_node

```

#### [XBOX 360 Joystick](#xbox-360-joystick "#xbox-360-joystick")

1. Connect XBOX 360 Joystick to the remote PC with Wireless Adapter or USB cable.
2. Open a terminal on **Remote PC**.
3. Launch the joystick node.

```
$ ros2 run joy joy_node

```
4. Open a new terminal and launch the teleoperation node.

```
$ ros2 run teleop_twist_joy teleop_node

```

### [Topic Monitor](#topic-monitor "#topic-monitor")

![](/assets/images/icon_unfold.png) **Read more about Topic Monitor**

In order to check topics of TurtleBot3, Use [rqt](http://wiki.ros.org/rqt "http://wiki.ros.org/rqt") provided by ROS, which is a Qt-based framework for GUI development for ROS. It is a tool displaying all topics of TurtleBot3 with a topic name, type, bandwidth, Hz, and value.

1. Run the rqt from PC with the command below. If the topic monitor window is not displayed, select the `plugin` -> `Topics` -> `Topic Monitor`.

```
$ rqt

```

![](/assets/images/platform/turtlebot3/ros2/rqt_1.png)
2. When the topic monitor loaded, the topic values are not monitored. Click the checkbox next to each topic to monitor the topic.  

![](/assets/images/platform/turtlebot3/ros2/rqt_2.png)
3. To see more detailed topic message, click the `▶` icon next to the checkbox.  

![](/assets/images/platform/turtlebot3/ros2/rqt_3.png)

* `/battery_state` indicates a message relating to the battery condition, such as the current battery voltage and remaining capacity.  

![](/assets/images/platform/turtlebot3/ros2/rqt_4.png)
* `/odom` indicates a message the odometry of the TurtleBot3. This topic has orientation and position by the encoder data.  

![](/assets/images/platform/turtlebot3/ros2/rqt_5.png)
* `/sensor_state` indicates a message the encoder values, battery and torque.  

![](/assets/images/platform/turtlebot3/ros2/rqt_6.png)
* `/scan` indicates a message all of the LDS data, such as angle\_max and min, range\_max and min, indicates, ranges.  

![](/assets/images/platform/turtlebot3/ros2/rqt_7.png)

## [Basic Operation](#basic-operation "#basic-operation")

### [Teleoperation](#teleoperation "#teleoperation")

**WARNING**: Make sure to run the [Bringup](/docs/en/platform/turtlebot3/bringup/#bringup "/docs/en/platform/turtlebot3/bringup/#bringup") from the TurtleBot3 SBC before teleoperation. Teleoperate the robot, and be careful when testing the robot on the table as the robot might fall.

The TurtleBot3 can be teleoperated by various remote controllers. Make sure that the necessary ROS packages are supported for your SBC and ROS version.

#### [Keyboard](#keyboard "#keyboard")

1. Open a terminal on **Remote PC**.
2. Run teleoperation node. Replace the `${TB3_MODEL}` with `burger` or `waffle` or `waffle_pi`, if TURTLEBOT3\_MODEL parameter is not predefined.

```
$ export TURTLEBOT3\_MODEL=${TB3\_MODEL}
$ ros2 run turtlebot3_teleop teleop_keyboard

```
3. If the node is successfully launched, the following instruction will be appeared to the terminal window.

```
Control Your Turtlebot3
Moving around
     w
 a   s   d
     x
w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)
space key, s : force stop
CTRL-C to quit

```

#### [RC-100](#rc100 "#rc100")

The settings for [ROBOTIS RC-100B](/docs/en/parts/communication/rc-100/ "/docs/en/parts/communication/rc-100/") is included in an OpenCR firmware for TurtleBot3. It can be used with the Bluetooth module [BT410](/docs/en/parts/communication/bt-410/ "/docs/en/parts/communication/bt-410/"). TurtleBot3 Waffle Pi includes this controller and Bluetooth modules. When using RC-100B, it is not necessary to execute a specific node because `turtlebot_core` node creates a `/cmd_vel` topic in the firmware directly connected to OpeCR.

![](/assets/images/platform/turtlebot3/example/rc100b_with_bt410.png)

1. Connect BT-410 to any of OpenCR UART ports.
2. Control TurtleBot3 with RC-100.

	* Up / Down : Increase or decrease linear velocity
	* Left / Right : Increase or decrease angular velocity

#### [PS3 Joystick](#ps3-joystick "#ps3-joystick")

1. Connect PS3 Joystick to **Remote PC** via Bluetooth or a USB cable.
2. Install `ds4drv` packages by using pip.

```
$ sudo pip install ds4drv

```
3. Launch the joystick node.

```
$ sudo ds4drv
$ ros2 run joy joy_node

```
4. Open a new terminal and launch the teleoperation node.

```
$ ros2 run teleop_twist_joy teleop_node

```

#### [XBOX 360 Joystick](#xbox-360-joystick "#xbox-360-joystick")

1. Connect XBOX 360 Joystick to the remote PC with Wireless Adapter or USB cable.
2. Open a terminal on **Remote PC**.
3. Launch the joystick node.

```
$ ros2 run joy joy_node

```
4. Open a new terminal and launch the teleoperation node.

```
$ ros2 run teleop_twist_joy teleop_node

```

### [Topic Monitor](#topic-monitor "#topic-monitor")

![](/assets/images/icon_unfold.png) **Read more about Topic Monitor**

In order to check topics of TurtleBot3, Use [rqt](http://wiki.ros.org/rqt "http://wiki.ros.org/rqt") provided by ROS, which is a Qt-based framework for GUI development for ROS. It is a tool displaying all topics of TurtleBot3 with a topic name, type, bandwidth, Hz, and value.

1. Run the rqt from PC with the command below. If the topic monitor window is not displayed, select the `plugin` -> `Topics` -> `Topic Monitor`.

```
$ rqt

```

![](/assets/images/platform/turtlebot3/ros2/rqt_1.png)
2. When the topic monitor loaded, the topic values are not monitored. Click the checkbox next to each topic to monitor the topic.  

![](/assets/images/platform/turtlebot3/ros2/rqt_2.png)
3. To see more detailed topic message, click the `▶` icon next to the checkbox.  

![](/assets/images/platform/turtlebot3/ros2/rqt_3.png)

* `/battery_state` indicates a message relating to the battery condition, such as the current battery voltage and remaining capacity.  

![](/assets/images/platform/turtlebot3/ros2/rqt_4.png)
* `/odom` indicates a message the odometry of the TurtleBot3. This topic has orientation and position by the encoder data.  

![](/assets/images/platform/turtlebot3/ros2/rqt_5.png)
* `/sensor_state` indicates a message the encoder values, battery and torque.  

![](/assets/images/platform/turtlebot3/ros2/rqt_6.png)
* `/scan` indicates a message all of the LDS data, such as angle\_max and min, range\_max and min, indicates, ranges.  

![](/assets/images/platform/turtlebot3/ros2/rqt_7.png)

## [Basic Operation](#basic-operation "#basic-operation")

### [Teleoperation](#teleoperation "#teleoperation")

**WARNING**: Make sure to run the [Bringup](/docs/en/platform/turtlebot3/bringup/#bringup "/docs/en/platform/turtlebot3/bringup/#bringup") from the TurtleBot3 SBC before teleoperation. Teleoperate the robot, and be careful when testing the robot on the table as the robot might fall.

The TurtleBot3 can be teleoperated by various remote controllers. Make sure that the necessary ROS packages are supported for your SBC and ROS version.

#### [Keyboard](#keyboard "#keyboard")

**TIP**: Before executing this command, you have to specify the model name of TurtleBot3. The `${TB3_MODEL}` is the name of the model you are using in `burger`, `waffle`, `waffle_pi`.

1. Launch `turtlebot3_teleop_key` node from Remote PC for the teleoperation using a keyboard.  

 Please use the proper keyword among `burger`, `waffle`, `waffle_pi` for the `TURTLEBOT3_MODEL` parameter.

```
> set TURTLEBOT3\_MODEL=waffle
> roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

```
2. If the node is successfully launched, the following instruction will be appeared to the terminal window.

```
Control Your Turtlebot3
Moving around
     w
 a   s   d
     x
w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop
CTRL-C to quit

```

#### [RC100](#rc100 "#rc100")

The settings for [ROBOTIS RC-100B](/docs/en/parts/communication/rc-100/ "/docs/en/parts/communication/rc-100/") controller is included in the OpenCR firmware for TurtleBot3 Burger, Waffle and Waffle Pi. This controller can be used with the Bluetooth module [BT410](/docs/en/parts/communication/bt-410/ "/docs/en/parts/communication/bt-410/"). The TurtleBot3 Waffle Pi includes the RC-100 controller and Bluetooth modules. When using RC-100, it is not necessary to execute a specific node because `turtlebot_core` node creates a `/cmd_vel` topic in the firmware directly connected to OpenCR.

![](/assets/images/platform/turtlebot3/example/rc100b_with_bt410.png)

1. Connect BT-410 to OpenCR UART1 port (as described [here](/docs/en/platform/turtlebot3/appendix_opencr1_0/ "/docs/en/platform/turtlebot3/appendix_opencr1_0/")).
2. Control TurtleBot3 with RC-100.

	* Up / Down : Increase or decrease linear velocity
	* Left / Right : Increase or decrease angular velocity

#### [Joysticks](#joysticks "#joysticks")

The Windows implementation of the Joystick control uses the [Open Source Simple DirectMedia Layer](https://www.libsdl.org/ "https://www.libsdl.org/"), which supports many tethered and wireless joysticks. The Joystick driver is currently (As of January 2020) deployed as a source package, which you need to clone into your catkin workspace.

```
> git clone -b init_windows https://github.com/ms-iot/joystick_drivers

```

### [Topic Monitor](#topic-monitor "#topic-monitor")

![](/assets/images/icon_unfold.png) **Read more about Topic Monitor**

In order to check the topics of TurtleBot3, we will use [rqt](http://wiki.ros.org/rqt "http://wiki.ros.org/rqt") provided by ROS. The rqt is a Qt-based framework for GUI development for ROS. The rqt is a tool that allows users to easily see the topic status by displaying all the topics in the topic list. There are topic names, types, bandwidth, Hz, value in GUI.

1. Run the rqt from PC with the command below. If the topic monitor window is not displayed, select the `plugin` -> `Topics` -> `Topic Monitor`.

```
> rqt

```

![](/assets/images/platform/turtlebot3/example/rqt_1.png)
2. When the topic monitor loaded, the topic values are not monitored. Click the checkbox next to each topic to monitor the topic.  

![](/assets/images/platform/turtlebot3/example/rqt_2.png)
3. To see more detailed topic message, click the `▶` icon next to the checkbox.  

![](/assets/images/platform/turtlebot3/example/rqt_3.png)

* `/battery_state` indicates a message relating to the battery condition, such as the current battery voltage and remaining capacity.  

![](/assets/images/platform/turtlebot3/example/rqt_4.png)
* `/diagnostics` indicates a message the status of the components connected to the TurtleBot3, such as a MPU9250, DYNAMIXEL-X, a HLS-LFCD-LDS, a battery and a OpenCR.  

![](/assets/images/platform/turtlebot3/example/rqt_5.png)
* `/odom` indicates a message the odometry of the TurtleBot3. This topic has orientation and position by the encoder data.  

![](/assets/images/platform/turtlebot3/example/rqt_6.png)
* `/sensor_state` indicates a message the encoder values, battery and torque.  

![](/assets/images/platform/turtlebot3/example/rqt_7.png)
* `/scan` indicates a message all of the LDS data, such as angle\_max and min, range\_max and min, indicates, ranges.  

![](/assets/images/platform/turtlebot3/example/rqt_8.png)

 Previous Page
Next Page 
