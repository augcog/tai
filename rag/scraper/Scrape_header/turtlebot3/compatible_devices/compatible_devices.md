
[Edit on GitHub](https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/more_info/compatible_devices.md "https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/more_info/compatible_devices.md") 

## [Compatible Devices](#compatible-devices "#compatible-devices")

* If you want to use other products instead of Computer and Sensors included in the basic configuration, please refer to the this page.

### [Computer](#computer "#computer")

* TurtleBot3’s main computer is `Raspberry Pi 3` (TurtleBot3 Burger and Waffle Pi) and `Intel Joule 570x` (TurtleBot3 Waffle). These SBCs (Single Board Computer) are enough to use the basic features of TurtleBot3, but users need to increase CPU performance, use GPU, or add RAM size for other purposes. This section describes how to replace the SBC.
* There are various types of SBC as shown in the following figure. The specification of each SBC is different. But if you can install Linux and ROS on the SBC you want to use, you can use that SBC as the main computer for TurtleBot3. In addition to SBC, [Intel NUC](https://www.intel.com/content/www/us/en/products/boards-kits/nuc.html "https://www.intel.com/content/www/us/en/products/boards-kits/nuc.html"), mini PC and small notebooks are available.

![](/assets/images/platform/turtlebot3/setup/sbcs.png)

* The TurtleBot3 development team has tested several SBCs. Here is a list of SBCs we tested:
	+ [Raspberry Pi 3](https://www.raspberrypi.org/products/ "https://www.raspberrypi.org/products/")
	+ [Intel Joule 570x](https://ark.intel.com/products/96414/Intel-Joule-570x-Developer-Kit "https://ark.intel.com/products/96414/Intel-Joule-570x-Developer-Kit")
	+ [DragonBoard 410c](https://developer.qualcomm.com/hardware/dragonboard-410c "https://developer.qualcomm.com/hardware/dragonboard-410c")
	+ [NVIDIA Jetson TX2](https://developer.nvidia.com/embedded/buy/jetson-tx2-devkit "https://developer.nvidia.com/embedded/buy/jetson-tx2-devkit")
	+ [UP Board](http://www.up-board.org/up/ "http://www.up-board.org/up/")
	+ [UP Core](http://www.up-board.org/upcore/ "http://www.up-board.org/upcore/")
	+ [LattePanda](https://www.lattepanda.com/ "https://www.lattepanda.com/")
	+ [ODROID-XU4](http://www.hardkernel.com/ "http://www.hardkernel.com/")

#### Hardware assembly

* Most of the SBCs can be assembled without problems using `PCB support`, which is built in TurtleBot3. For reference, you can purchase additional parts such as PCB support ([link](http://www.robotis-shop-en.com/?act=shop_en.goods_view&GS=3284&GC=GD070003 "http://www.robotis-shop-en.com/?act=shop_en.goods_view&GS=3284&GC=GD070003")), download [files](http://www.robotis.com/service/download.php?no=676 "http://www.robotis.com/service/download.php?no=676") shared with Onshape, and print them using a 3D printer.

![](/assets/images/platform/turtlebot3/setup/pcb_support.png)

* You can fix SBC on the waffle-plate of TurtleBot3 using the PCB support in the fixing hole of the SBC to be used as shown in the following figure.

![](/assets/images/platform/turtlebot3/setup/pcb_support_and_sbc.png)

#### Power supply

* Hardware assembly of the SBC is simple. But the power supply is not simple. You need to modify the existing power cable or make a new power cable to match the `power cable` of the computer you are going to use.
* As a basic part of TurtleBot3, the following `power cable` is provided. The left figure is for Raspberry Pi and the right figure is for Intel Joule 570x. The power cable must be made to match the power specifications of the computer you are using. [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") has both a 5V (4A) power and a 12V (1A) power, which are commonly used in SBCs.

![](/assets/images/platform/turtlebot3/setup/power_cable.png)

* The power source for the SBC is the three connectors on the left in the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") pinmap below.

![](/assets/images/parts/controller/opencr10/opencr_pinout.png)

### [Sensors](#sensors "#sensors")

* `TurtleBot3 Burger` uses enhanced [360° LiDAR](/docs/en/platform/turtlebot3/appendix_lds_01/ "/docs/en/platform/turtlebot3/appendix_lds_01/"), [9-Axis Inertial Measurement Unit](/docs/en/platform/turtlebot3/appendix_opencr1_0/#specifications "/docs/en/platform/turtlebot3/appendix_opencr1_0/#specifications") and [precise encoder](/docs/en/platform/turtlebot3/appendix_dynamixel/ "/docs/en/platform/turtlebot3/appendix_dynamixel/") for your research and development. `TurtleBot3 Waffle` is equipped with an identical 360° LiDAR as well but additionally proposes a powerful [Intel® RealSense™](/docs/en/platform/turtlebot3/appendix_realsense/ "/docs/en/platform/turtlebot3/appendix_realsense/") with the recognition SDK. `TurtleBot3 Waffle Pi` uses high utilized [Raspberry Pi Camera](/docs/en/platform/turtlebot3/appendix_raspi_cam/ "/docs/en/platform/turtlebot3/appendix_raspi_cam/"). This will be the best hardware solution for making a mobile robot.
* If you use an additional sensor, you can use it after attaching the sensor to the robot. The ROS provides a development environment in which drivers and libraries of the aforementioned sensors can be used. Not all sensors are supported by ROS package, but more and more sensor related packages are increasing.

![](/assets/images/platform/turtlebot3/setup/sensors.png)

* If you are looking for a new sensor, check out [Sensors page](http://wiki.ros.org/Sensors "http://wiki.ros.org/Sensors") of ROS Wiki to find the sensor and related ROS packages you want.
* If you are using an analog sensor connected to the embedded board, you can use it with OpenCR. If you need to use an analog sensor other than USB or Ethernet communication, refer to [Additional Sensors](/docs/en/platform/turtlebot3/additional_sensors/ "/docs/en/platform/turtlebot3/additional_sensors/") page.

 Previous Page
Next Page 
