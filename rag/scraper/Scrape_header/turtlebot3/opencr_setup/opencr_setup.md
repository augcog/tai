
[Edit on GitHub](https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/quick_start/opencr_setup.md "https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/quick_start/opencr_setup.md") 

Kinetic 
Melodic
Noetic
Dashing
Foxy
Humble
Windows

## [OpenCR Setup](#opencr-setup "#opencr-setup")

1. Connect the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") to the Rasbperry Pi using the micro USB cable.
2. Install required packages on the Raspberry Pi to upload the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") firmware.

```
$ sudo dpkg --add-architecture armhf
$ sudo apt-get update
$ sudo apt-get install libc6:armhf

```
3. Depending on the platform, use either `burger` or `waffle` for the **OPENCR\_MODEL** name.

```
$ export OPENCR\_PORT=/dev/ttyACM0
$ export OPENCR\_MODEL=burger
$ rm -rf ./opencr_update.tar.bz2

```
4. Download the firmware and loader, then extract the file.

```
$ wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2 
$ tar -xvf opencr_update.tar.bz2 

```
5. Upload firmware to the OpenCR.

```
$ cd ./opencr_update
$ ./update.sh $OPENCR\_PORT $OPENCR\_MODEL.opencr

```
6. A successful firmware upload for TurtleBot3 Burger will look like below.  

![](/assets/images/platform/turtlebot3/opencr/shell01.png)
7. If firmware upload fails, try uploading with the recovery mode. Below sequence activates the recovery mode of OpenCR. Under the recovery mode, the `STATUS` led of [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") will blink periodically.
	* Hold down the `PUSH SW2` button.
	* Press the `Reset` button.
	* Release the `Reset` button.
	* Release the `PUSH SW2` button.
	 ![](/assets/images/parts/controller/opencr10/bootloader_19.png)

![](/assets/images/icon_unfold.png) Click here to expand more details about the firmware upload using **Arduino IDE**

Please be aware that [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") board manager **does not support Arduino IDE on ARM based SBC such as Raspberry Pi or NVidia Jetson**.  

In order to upload the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") firmware using Arduino IDE, please follow the below instructions on your PC.

1. If you are using Linux, please configure the USB port for OpenCR. For other OS(OSX or Windows), you can skip this step.

```
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/99-opencr-cdc.rules
$ sudo cp ./99-opencr-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
$ sudo apt install libncurses5-dev:i386

```
2. Install Arduino IDE.
	* [Download the latest Arduino IDE](https://www.arduino.cc/en/software "https://www.arduino.cc/en/software")
3. After completing the installation, run Arduino IDE.
4. Press `Ctrl` + `,` to open the Preferences menu
5. Enter below address in the `Additional Boards Manager URLs`.

```
https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json

```

![](/assets/images/platform/turtlebot3/preparation/ide1.png)
6. Open the TurtleBot3 firmware. Depending on your platform, please select the correct firmware.
	* Burger : ***File > Examples > turtlebot3 > turtlebot3\_burger > turtlebot3\_core***
	* Waffle/Waffle Pi : ***File > Examples > turtlebot3 > turtlebot3\_waffle > turtlebot3\_core***
7. Connect [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") to the PC and Select ***OpenCR > OpenCR Board*** from ***Tools > Board*** menu.
8. Select the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") connected USB port from ***Tools > Port*** menu.
9. Upload the TurtleBot3 firmware sketch with `Ctrl` + `U` or the upload icon.  

![](/assets/images/platform/turtlebot3/opencr/o2.png)  

![](/assets/images/platform/turtlebot3/opencr/o3.png)
10. If firmware upload fails, try uploading with the recovery mode. Below sequence activates the recovery mode of OpenCR. Under the recovery mode, the `STATUS` led of [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") will blink periodically.
	* Hold down the `PUSH SW2` button.
	* Press the `Reset` button.
	* Release the `Reset` button.
	* Release the `PUSH SW2` button.
	 ![](/assets/images/parts/controller/opencr10/bootloader_19.png)

### [OpenCR Test](#opencr-test "#opencr-test")

**NOTE**: If the wheels do not move while performing OpenCR Test instruction, make sure to see “**[Setup DYNAMIXELs for TurtleBot3](/docs/en/platform/turtlebot3/faq/#setup-dynamixels-for-turtlebot3 "/docs/en/platform/turtlebot3/faq/#setup-dynamixels-for-turtlebot3")**” section to update the DYNAMIXEL’s configuration for use of TurtleBot3.

You can use `PUSH SW 1` and `PUSH SW 2` buttons to see whether your robot has been properly assembled. This process tests the left and right DYNAMIXEL’s and the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") board.

![](/assets/images/platform/turtlebot3/opencr/opencr_models.png)

1. After assembling TurtleBot3, connect the power to [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") and turn on the power switch of OpenCR. The red `Power LED` will be turned on.
2. Place the robot on the flat ground in a wide open area. For the test, safety radius of 1 meter (40 inches) is recommended.
3. Press and hold `PUSH SW 1` for a few seconds to command the robot to move 30 centimeters (about 12 inches) forward.
4. Press and hold `PUSH SW 2` for a few seconds to command the robot to rotate 180 degrees in place.

## [OpenCR Setup](#opencr-setup "#opencr-setup")

1. Connect the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") to the Rasbperry Pi using the micro USB cable.
2. Install required packages on the Raspberry Pi to upload the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") firmware.

```
$ sudo dpkg --add-architecture armhf
$ sudo apt-get update
$ sudo apt-get install libc6:armhf

```
3. Depending on the platform, use either `burger` or `waffle` for the **OPENCR\_MODEL** name.

```
$ export OPENCR\_PORT=/dev/ttyACM0
$ export OPENCR\_MODEL=burger
$ rm -rf ./opencr_update.tar.bz2

```
4. Download the firmware and loader, then extract the file.

```
$ wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2 
$ tar -xvf opencr_update.tar.bz2 

```
5. Upload firmware to the OpenCR.

```
$ cd ./opencr_update
$ ./update.sh $OPENCR\_PORT $OPENCR\_MODEL.opencr

```
6. A successful firmware upload for TurtleBot3 Burger will look like below.  

![](/assets/images/platform/turtlebot3/opencr/shell01.png)
7. If firmware upload fails, try uploading with the recovery mode. Below sequence activates the recovery mode of OpenCR. Under the recovery mode, the `STATUS` led of [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") will blink periodically.
	* Hold down the `PUSH SW2` button.
	* Press the `Reset` button.
	* Release the `Reset` button.
	* Release the `PUSH SW2` button.
	 ![](/assets/images/parts/controller/opencr10/bootloader_19.png)

![](/assets/images/icon_unfold.png) Click here to expand more details about the firmware upload using **Arduino IDE**

Please be aware that [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") board manager **does not support Arduino IDE on ARM based SBC such as Raspberry Pi or NVidia Jetson**.  

In order to upload the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") firmware using Arduino IDE, please follow the below instructions on your PC.

1. If you are using Linux, please configure the USB port for OpenCR. For other OS(OSX or Windows), you can skip this step.

```
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/99-opencr-cdc.rules
$ sudo cp ./99-opencr-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
$ sudo apt install libncurses5-dev:i386

```
2. Install Arduino IDE.
	* [Download the latest Arduino IDE](https://www.arduino.cc/en/software "https://www.arduino.cc/en/software")
3. After completing the installation, run Arduino IDE.
4. Press `Ctrl` + `,` to open the Preferences menu
5. Enter below address in the `Additional Boards Manager URLs`.

```
https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json

```

![](/assets/images/platform/turtlebot3/preparation/ide1.png)
6. Open the TurtleBot3 firmware. Depending on your platform, please select the correct firmware.
	* Burger : ***File > Examples > turtlebot3 > turtlebot3\_burger > turtlebot3\_core***
	* Waffle/Waffle Pi : ***File > Examples > turtlebot3 > turtlebot3\_waffle > turtlebot3\_core***
7. Connect [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") to the PC and Select ***OpenCR > OpenCR Board*** from ***Tools > Board*** menu.
8. Select the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") connected USB port from ***Tools > Port*** menu.
9. Upload the TurtleBot3 firmware sketch with `Ctrl` + `U` or the upload icon.  

![](/assets/images/platform/turtlebot3/opencr/o2.png)  

![](/assets/images/platform/turtlebot3/opencr/o3.png)
10. If firmware upload fails, try uploading with the recovery mode. Below sequence activates the recovery mode of OpenCR. Under the recovery mode, the `STATUS` led of [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") will blink periodically.
	* Hold down the `PUSH SW2` button.
	* Press the `Reset` button.
	* Release the `Reset` button.
	* Release the `PUSH SW2` button.
	 ![](/assets/images/parts/controller/opencr10/bootloader_19.png)

### [OpenCR Test](#opencr-test "#opencr-test")

**NOTE**: If the wheels do not move while performing OpenCR Test instruction, make sure to see “**[Setup DYNAMIXELs for TurtleBot3](/docs/en/platform/turtlebot3/faq/#setup-dynamixels-for-turtlebot3 "/docs/en/platform/turtlebot3/faq/#setup-dynamixels-for-turtlebot3")**” section to update the DYNAMIXEL’s configuration for use of TurtleBot3.

You can use `PUSH SW 1` and `PUSH SW 2` buttons to see whether your robot has been properly assembled. This process tests the left and right DYNAMIXEL’s and the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") board.

![](/assets/images/platform/turtlebot3/opencr/opencr_models.png)

1. After assembling TurtleBot3, connect the power to [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") and turn on the power switch of OpenCR. The red `Power LED` will be turned on.
2. Place the robot on the flat ground in a wide open area. For the test, safety radius of 1 meter (40 inches) is recommended.
3. Press and hold `PUSH SW 1` for a few seconds to command the robot to move 30 centimeters (about 12 inches) forward.
4. Press and hold `PUSH SW 2` for a few seconds to command the robot to rotate 180 degrees in place.

## [OpenCR Setup](#opencr-setup "#opencr-setup")

1. Connect the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") to the Rasbperry Pi using the micro USB cable.
2. Install required packages on the Raspberry Pi to upload the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") firmware.

```
$ sudo dpkg --add-architecture armhf
$ sudo apt-get update
$ sudo apt-get install libc6:armhf

```
3. Depending on the platform, use either `burger` or `waffle` for the **OPENCR\_MODEL** name.

```
$ export OPENCR\_PORT=/dev/ttyACM0
$ export OPENCR\_MODEL=burger_noetic
$ rm -rf ./opencr_update.tar.bz2

```
4. Download the firmware and loader, then extract the file.

```
$ wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2 
$ tar -xvf opencr_update.tar.bz2 

```
5. Upload firmware to the OpenCR.

```
$ cd ./opencr_update
$ ./update.sh $OPENCR\_PORT $OPENCR\_MODEL.opencr

```
6. A successful firmware upload for TurtleBot3 Burger will look like below.  

![](/assets/images/platform/turtlebot3/opencr/shell01.png)
7. If firmware upload fails, try uploading with the recovery mode. Below sequence activates the recovery mode of OpenCR. Under the recovery mode, the `STATUS` led of [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") will blink periodically.
	* Hold down the `PUSH SW2` button.
	* Press the `Reset` button.
	* Release the `Reset` button.
	* Release the `PUSH SW2` button.
	 ![](/assets/images/parts/controller/opencr10/bootloader_19.png)

![](/assets/images/icon_unfold.png) Click here to expand more details about the firmware upload using **Arduino IDE**

Please be aware that [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") board manager **does not support Arduino IDE on ARM based SBC such as Raspberry Pi or NVidia Jetson**.  

In order to upload the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") firmware using Arduino IDE, please follow the below instructions on your PC.

1. If you are using Linux, please configure the USB port for OpenCR. For other OS(OSX or Windows), you can skip this step.

```
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/99-opencr-cdc.rules
$ sudo cp ./99-opencr-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
$ sudo apt install libncurses5-dev:i386

```
2. Install Arduino IDE.
	* [Download the latest Arduino IDE](https://www.arduino.cc/en/software "https://www.arduino.cc/en/software")
3. After completing the installation, run Arduino IDE.
4. Press `Ctrl` + `,` to open the Preferences menu
5. Enter below address in the `Additional Boards Manager URLs`.

```
https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json

```

![](/assets/images/platform/turtlebot3/preparation/ide1.png)
6. Open the TurtleBot3 firmware. Depending on your platform, please select the correct firmware.
	* Burger : ***File > Examples > turtlebot3 > turtlebot3\_burger > turtlebot3\_core***
	* Waffle/Waffle Pi : ***File > Examples > turtlebot3 > turtlebot3\_waffle > turtlebot3\_core***
7. Open the `turtlebot3_core_config.h` and uncomment the **NOETIC\_SUPPORT** defintion in the line 21.
8. Connect [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") to the PC and Select ***OpenCR > OpenCR Board*** from ***Tools > Board*** menu.
9. Select the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") connected USB port from ***Tools > Port*** menu.
10. Upload the TurtleBot3 firmware sketch with `Ctrl` + `U` or the upload icon.  

![](/assets/images/platform/turtlebot3/opencr/o2.png)  

![](/assets/images/platform/turtlebot3/opencr/o3.png)
11. If firmware upload fails, try uploading with the recovery mode. Below sequence activates the recovery mode of OpenCR. Under the recovery mode, the `STATUS` led of [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") will blink periodically.
	* Hold down the `PUSH SW2` button.
	* Press the `Reset` button.
	* Release the `Reset` button.
	* Release the `PUSH SW2` button.
	 ![](/assets/images/parts/controller/opencr10/bootloader_19.png)

### [OpenCR Test](#opencr-test "#opencr-test")

**NOTE**: If the wheels do not move while performing OpenCR Test instruction, make sure to see “**[Setup DYNAMIXELs for TurtleBot3](/docs/en/platform/turtlebot3/faq/#setup-dynamixels-for-turtlebot3 "/docs/en/platform/turtlebot3/faq/#setup-dynamixels-for-turtlebot3")**” section to update the DYNAMIXEL’s configuration for use of TurtleBot3.

You can use `PUSH SW 1` and `PUSH SW 2` buttons to see whether your robot has been properly assembled. This process tests the left and right DYNAMIXEL’s and the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") board.

![](/assets/images/platform/turtlebot3/opencr/opencr_models.png)

1. After assembling TurtleBot3, connect the power to [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") and turn on the power switch of OpenCR. The red `Power LED` will be turned on.
2. Place the robot on the flat ground in a wide open area. For the test, safety radius of 1 meter (40 inches) is recommended.
3. Press and hold `PUSH SW 1` for a few seconds to command the robot to move 30 centimeters (about 12 inches) forward.
4. Press and hold `PUSH SW 2` for a few seconds to command the robot to rotate 180 degrees in place.

## [OpenCR Setup](#opencr-setup "#opencr-setup")

1. Connect the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") to the Rasbperry Pi using the micro USB cable.
2. Install required packages on the Raspberry Pi to upload the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") firmware.

```
$ sudo dpkg --add-architecture armhf
$ sudo apt-get update
$ sudo apt-get install libc6:armhf

```
3. Depending on the platform, use either `burger` or `waffle` for the **OPENCR\_MODEL** name.

```
$ export OPENCR\_PORT=/dev/ttyACM0
$ export OPENCR\_MODEL=burger
$ rm -rf ./opencr_update.tar.bz2

```
4. Download the firmware and loader, then extract the file.

```
$ wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
$ tar -xvf ./opencr_update.tar.bz2

```
5. Upload firmware to the OpenCR.

```
$ cd ~/opencr_update
$ ./update.sh $OPENCR\_PORT $OPENCR\_MODEL.opencr

```
6. A successful firmware upload for TurtleBot3 Burger will look like below.  

![](/assets/images/platform/turtlebot3/opencr/shell01.png)
7. If firmware upload fails, try uploading with the recovery mode. Below sequence activates the recovery mode of OpenCR. Under the recovery mode, the `STATUS` led of [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") will blink periodically.
	* Hold down the `PUSH SW2` button.
	* Press the `Reset` button.
	* Release the `Reset` button.
	* Release the `PUSH SW2` button.
	 ![](/assets/images/parts/controller/opencr10/bootloader_19.png)

![](/assets/images/icon_unfold.png) Click here to expand more details about the firmware upload using **Arduino IDE**

Please be aware that [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") board manager **does not support Arduino IDE on ARM based SBC such as Raspberry Pi or NVidia Jetson**.  

In order to upload the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") firmware using Arduino IDE, please follow the below instructions on your PC.

1. If you are using Linux, please configure the USB port for OpenCR. For other OS(OSX or Windows), you can skip this step.

```
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/99-opencr-cdc.rules
$ sudo cp ./99-opencr-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
$ sudo apt install libncurses5-dev:i386

```
2. Install Arduino IDE.
	* [Download the latest Arduino IDE](https://www.arduino.cc/en/software "https://www.arduino.cc/en/software")
3. After completing the installation, run Arduino IDE.
4. Press `Ctrl` + `,` to open the Preferences menu
5. Enter below address in the `Additional Boards Manager URLs`.

```
https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json

```

![](/assets/images/platform/turtlebot3/preparation/ide1.png)
6. Open the TurtleBot3 firmware. Depending on your platform, please select the correct firmware.
	* Burger : ***File > Examples > Turtlebot3 ROS2 > turtlebot3\_burger***
	* Waffle/Waffle Pi : ***File > Examples > Turtlebot3 ROS2 > turtlebot3\_waffle***
7. Connect [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") to the PC and Select ***OpenCR > OpenCR Board*** from ***Tools > Board*** menu.
8. Select the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") connected USB port from ***Tools > Port*** menu.
9. Upload the TurtleBot3 firmware sketch with `Ctrl` + `U` or the upload icon.  

![](/assets/images/platform/turtlebot3/opencr/o2.png)  

![](/assets/images/platform/turtlebot3/opencr/o3.png)
10. If firmware upload fails, try uploading with the recovery mode. Below sequence activates the recovery mode of OpenCR. Under the recovery mode, the `STATUS` led of [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") will blink periodically.
	* Hold down the `PUSH SW2` button.
	* Press the `Reset` button.
	* Release the `Reset` button.
	* Release the `PUSH SW2` button.
	 ![](/assets/images/parts/controller/opencr10/bootloader_19.png)

### [OpenCR Test](#opencr-test "#opencr-test")

**NOTE**: If the wheels do not move while performing OpenCR Test instruction, make sure to see “**[Setup DYNAMIXELs for TurtleBot3](/docs/en/platform/turtlebot3/faq/#setup-dynamixels-for-turtlebot3 "/docs/en/platform/turtlebot3/faq/#setup-dynamixels-for-turtlebot3")**” section to update the DYNAMIXEL’s configuration for use of TurtleBot3.

You can use `PUSH SW 1` and `PUSH SW 2` buttons to see whether your robot has been properly assembled. This process tests the left and right DYNAMIXEL’s and the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") board.

![](/assets/images/platform/turtlebot3/opencr/opencr_models.png)

1. After assembling TurtleBot3, connect the power to [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") and turn on the power switch of OpenCR. The red `Power LED` will be turned on.
2. Place the robot on the flat ground in a wide open area. For the test, safety radius of 1 meter (40 inches) is recommended.
3. Press and hold `PUSH SW 1` for a few seconds to command the robot to move 30 centimeters (about 12 inches) forward.
4. Press and hold `PUSH SW 2` for a few seconds to command the robot to rotate 180 degrees in place.

## [OpenCR Setup](#opencr-setup "#opencr-setup")

1. Connect the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") to the Rasbperry Pi using the micro USB cable.
2. Install required packages on the Raspberry Pi to upload the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") firmware.

```
$ sudo dpkg --add-architecture armhf
$ sudo apt update
$ sudo apt install libc6:armhf

```
3. Depending on the platform, use either `burger` or `waffle` for the **OPENCR\_MODEL** name.

```
$ export OPENCR\_PORT=/dev/ttyACM0
$ export OPENCR\_MODEL=burger
$ rm -rf ./opencr_update.tar.bz2

```
4. Download the firmware and loader, then extract the file.

```
$ wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
$ tar -xvf ./opencr_update.tar.bz2

```
5. Upload firmware to the OpenCR.

```
$ cd ~/opencr_update
$ ./update.sh $OPENCR\_PORT $OPENCR\_MODEL.opencr

```
6. A successful firmware upload for TurtleBot3 Burger will look like below.  

![](/assets/images/platform/turtlebot3/opencr/shell01.png)
7. If firmware upload fails, try uploading with the recovery mode. Below sequence activates the recovery mode of OpenCR. Under the recovery mode, the `STATUS` led of [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") will blink periodically.
	* Hold down the `PUSH SW2` button.
	* Press the `Reset` button.
	* Release the `Reset` button.
	* Release the `PUSH SW2` button.

![](/assets/images/parts/controller/opencr10/bootloader_19.png)

![](/assets/images/icon_unfold.png) Click here to expand more details about the firmware upload using **Arduino IDE**

Please be aware that [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") board manager **does not support Arduino IDE on ARM based SBC such as Raspberry Pi or NVidia Jetson**.  

In order to upload the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") firmware using Arduino IDE, please follow the below instructions on your PC.

1. If you are using Linux, please configure the USB port for OpenCR. For other OS(OSX or Windows), you can skip this step.

```
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/99-opencr-cdc.rules
$ sudo cp ./99-opencr-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
$ sudo apt install libncurses5-dev:i386

```
2. Install Arduino IDE.
	* [Download the latest Arduino IDE](https://www.arduino.cc/en/software "https://www.arduino.cc/en/software")
3. After completing the installation, run Arduino IDE.
4. Press `Ctrl` + `,` to open the Preferences menu
5. Enter below address in the `Additional Boards Manager URLs`.

```
https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json

```

![](/assets/images/platform/turtlebot3/preparation/ide1.png)
6. Open the TurtleBot3 firmware. Depending on your platform, please select the correct firmware.
	* Burger : ***File > Examples > Turtlebot3 ROS2 > turtlebot3\_burger***
	* Waffle/Waffle Pi : ***File > Examples > Turtlebot3 ROS2 > turtlebot3\_waffle***
7. Connect [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") to the PC and Select ***OpenCR > OpenCR Board*** from ***Tools > Board*** menu.
8. Select the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") connected USB port from ***Tools > Port*** menu.
9. Upload the TurtleBot3 firmware sketch with `Ctrl` + `U` or the upload icon.  

![](/assets/images/platform/turtlebot3/opencr/o2.png)  

![](/assets/images/platform/turtlebot3/opencr/o3.png)
10. If firmware upload fails, try uploading with the recovery mode. Below sequence activates the recovery mode of OpenCR. Under the recovery mode, the `STATUS` led of [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") will blink periodically.
	* Hold down the `PUSH SW2` button.
	* Press the `Reset` button.
	* Release the `Reset` button.
	* Release the `PUSH SW2` button.
	 ![](/assets/images/parts/controller/opencr10/bootloader_19.png)

### [OpenCR Test](#opencr-test "#opencr-test")

**NOTE**: If the wheels do not move while performing OpenCR Test instruction, make sure to see “**[Setup DYNAMIXELs for TurtleBot3](/docs/en/platform/turtlebot3/faq/#setup-dynamixels-for-turtlebot3 "/docs/en/platform/turtlebot3/faq/#setup-dynamixels-for-turtlebot3")**” section to update the DYNAMIXEL’s configuration for use of TurtleBot3.

You can use `PUSH SW 1` and `PUSH SW 2` buttons to see whether your robot has been properly assembled. This process tests the left and right DYNAMIXEL’s and the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") board.

![](/assets/images/platform/turtlebot3/opencr/opencr_models.png)

1. After assembling TurtleBot3, connect the power to [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") and turn on the power switch of OpenCR. The red `Power LED` will be turned on.
2. Place the robot on the flat ground in a wide open area. For the test, safety radius of 1 meter (40 inches) is recommended.
3. Press and hold `PUSH SW 1` for a few seconds to command the robot to move 30 centimeters (about 12 inches) forward.
4. Press and hold `PUSH SW 2` for a few seconds to command the robot to rotate 180 degrees in place.

## [OpenCR Setup](#opencr-setup "#opencr-setup")

1. Connect the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") to the Rasbperry Pi using the micro USB cable.
2. Install required packages on the Raspberry Pi to upload the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") firmware.

```
$ sudo dpkg --add-architecture armhf
$ sudo apt update
$ sudo apt install libc6:armhf

```
3. Depending on the platform, use either `burger` or `waffle` for the **OPENCR\_MODEL** name.

```
$ export OPENCR\_PORT=/dev/ttyACM0
$ export OPENCR\_MODEL=burger
$ rm -rf ./opencr_update.tar.bz2

```
4. Download the firmware and loader, then extract the file.

```
$ wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
$ tar -xvf ./opencr_update.tar.bz2

```
5. Upload firmware to the OpenCR.

```
$ cd ~/opencr_update
$ ./update.sh $OPENCR\_PORT $OPENCR\_MODEL.opencr

```
6. A successful firmware upload for TurtleBot3 Burger will look like below.  

![](/assets/images/platform/turtlebot3/opencr/shell01.png)
7. If firmware upload fails, try uploading with the recovery mode. Below sequence activates the recovery mode of OpenCR. Under the recovery mode, the `STATUS` led of [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") will blink periodically.
	* Hold down the `PUSH SW2` button.
	* Press the `Reset` button.
	* Release the `Reset` button.
	* Release the `PUSH SW2` button.

![](/assets/images/parts/controller/opencr10/bootloader_19.png)

![](/assets/images/icon_unfold.png) Click here to expand more details about the firmware upload using **Arduino IDE**

Please be aware that [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") board manager **does not support Arduino IDE on ARM based SBC such as Raspberry Pi or NVidia Jetson**.  

In order to upload the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") firmware using Arduino IDE, please follow the below instructions on your PC.

1. If you are using Linux, please configure the USB port for OpenCR. For other OS(OSX or Windows), you can skip this step.

```
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/99-opencr-cdc.rules
$ sudo cp ./99-opencr-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
$ sudo apt install libncurses5-dev:i386

```
2. Install Arduino IDE.
	* [Download the latest Arduino IDE](https://www.arduino.cc/en/software "https://www.arduino.cc/en/software")
3. After completing the installation, run Arduino IDE.
4. Press `Ctrl` + `,` to open the Preferences menu
5. Enter below address in the `Additional Boards Manager URLs`.

```
https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json

```

![](/assets/images/platform/turtlebot3/preparation/ide1.png)
6. Open the TurtleBot3 firmware. Depending on your platform, please select the correct firmware.
	* Burger : ***File > Examples > Turtlebot3 ROS2 > turtlebot3\_burger***
	* Waffle/Waffle Pi : ***File > Examples > Turtlebot3 ROS2 > turtlebot3\_waffle***
7. Connect [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") to the PC and Select ***OpenCR > OpenCR Board*** from ***Tools > Board*** menu.
8. Select the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") connected USB port from ***Tools > Port*** menu.
9. Upload the TurtleBot3 firmware sketch with `Ctrl` + `U` or the upload icon.  

![](/assets/images/platform/turtlebot3/opencr/o2.png)  

![](/assets/images/platform/turtlebot3/opencr/o3.png)
10. If firmware upload fails, try uploading with the recovery mode. Below sequence activates the recovery mode of OpenCR. Under the recovery mode, the `STATUS` led of [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") will blink periodically.
	* Hold down the `PUSH SW2` button.
	* Press the `Reset` button.
	* Release the `Reset` button.
	* Release the `PUSH SW2` button.
	 ![](/assets/images/parts/controller/opencr10/bootloader_19.png)

### [OpenCR Test](#opencr-test "#opencr-test")

**NOTE**: If the wheels do not move while performing OpenCR Test instruction, make sure to see “**[Setup DYNAMIXELs for TurtleBot3](/docs/en/platform/turtlebot3/faq/#setup-dynamixels-for-turtlebot3 "/docs/en/platform/turtlebot3/faq/#setup-dynamixels-for-turtlebot3")**” section to update the DYNAMIXEL’s configuration for use of TurtleBot3.

You can use `PUSH SW 1` and `PUSH SW 2` buttons to see whether your robot has been properly assembled. This process tests the left and right DYNAMIXEL’s and the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") board.

![](/assets/images/platform/turtlebot3/opencr/opencr_models.png)

1. After assembling TurtleBot3, connect the power to [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") and turn on the power switch of OpenCR. The red `Power LED` will be turned on.
2. Place the robot on the flat ground in a wide open area. For the test, safety radius of 1 meter (40 inches) is recommended.
3. Press and hold `PUSH SW 1` for a few seconds to command the robot to move 30 centimeters (about 12 inches) forward.
4. Press and hold `PUSH SW 2` for a few seconds to command the robot to rotate 180 degrees in place.

## [OpenCR Setup](#opencr-setup "#opencr-setup")

Please be aware that [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") board manager **does not support Arduino IDE on ARM based SBC such as Raspberry Pi or NVidia Jetson**.  

In order to upload the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") firmware using Arduino IDE, please follow the below instructions on your PC.

1. Install Arduino IDE.
	* [Download the latest Arduino IDE](https://www.arduino.cc/en/software "https://www.arduino.cc/en/software")
2. After completing the installation, run Arduino IDE.
3. Press `Ctrl` + `,` to open the Preferences menu
4. Enter below address in the `Additional Boards Manager URLs`.

```
https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json

```

![](/assets/images/platform/turtlebot3/preparation/ide1.png)
5. Open the TurtleBot3 firmware. Depending on your platform, please select the correct firmware.
	* Burger : ***File > Examples > turtlebot3 > turtlebot3\_burger > turtlebot3\_core***
	* Waffle/Waffle Pi : ***File > Examples > turtlebot3 > turtlebot3\_waffle > turtlebot3\_core***
6. Connect [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") to the PC and Select ***OpenCR > OpenCR Board*** from ***Tools > Board*** menu.
7. Select the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") connected USB port from ***Tools > Port*** menu.
8. Upload the TurtleBot3 firmware sketch with `Ctrl` + `U` or the upload icon.  

![](/assets/images/platform/turtlebot3/opencr/o2.png)  

![](/assets/images/platform/turtlebot3/opencr/o3.png)
9. If firmware upload fails, try uploading with the recovery mode. Below sequence activates the recovery mode of OpenCR. Under the recovery mode, the `STATUS` led of [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") will blink periodically.
	* Hold down the `PUSH SW2` button.
	* Press the `Reset` button.
	* Release the `Reset` button.
	* Release the `PUSH SW2` button.
	 ![](/assets/images/parts/controller/opencr10/bootloader_19.png)

### [OpenCR Test](#opencr-test "#opencr-test")

**NOTE**: If the wheels do not move while performing OpenCR Test instruction, make sure to see “**[Setup DYNAMIXELs for TurtleBot3](/docs/en/platform/turtlebot3/faq/#setup-dynamixels-for-turtlebot3 "/docs/en/platform/turtlebot3/faq/#setup-dynamixels-for-turtlebot3")**” section to update the DYNAMIXEL’s configuration for use of TurtleBot3.

You can use `PUSH SW 1` and `PUSH SW 2` buttons to see whether your robot has been properly assembled. This process tests the left and right DYNAMIXEL’s and the [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") board.

![](/assets/images/platform/turtlebot3/opencr/opencr_models.png)

1. After assembling TurtleBot3, connect the power to [OpenCR](/docs/en/parts/controller/opencr10/ "/docs/en/parts/controller/opencr10/") and turn on the power switch of OpenCR. The red `Power LED` will be turned on.
2. Place the robot on the flat ground in a wide open area. For the test, safety radius of 1 meter (40 inches) is recommended.
3. Press and hold `PUSH SW 1` for a few seconds to command the robot to move 30 centimeters (about 12 inches) forward.
4. Press and hold `PUSH SW 2` for a few seconds to command the robot to rotate 180 degrees in place.

 Previous Page
Next Page 
