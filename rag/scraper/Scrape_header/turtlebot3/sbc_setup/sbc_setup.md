
[Edit on GitHub](https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/quick_start/sbc_setup.md "https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/quick_start/sbc_setup.md") 

Kinetic 
Melodic
Noetic
Dashing
Foxy
Humble
Windows

## [SBC Setup](#sbc-setup "#sbc-setup")

**WARNING**

* `Raspberry Pi 4`, `Jetson Nano` do not support ROS Kinetic.
* This process may take long time. Please do not use battery while following this section.
* An HDMI monitor and input devices such as a keyboard and a mouse will be required to complete this setup.

If you are using **Intel Joule**, please refer to [Intel Joule Setup](/docs/en/popup/turtlebot3/joule_setup "/docs/en/popup/turtlebot3/joule_setup") instruction.

### [Prepare microSD Card and Reader](#prepare-microsd-card-and-reader "#prepare-microsd-card-and-reader")

If you PC do not have a microSD slot, please use a microSD card reader to burn the recovery image.  

![](/assets/images/platform/turtlebot3/setup/micro_sd_reader.png)

### [Download TurtleBot3 SBC Image](#download-turtlebot3-sbc-image "#download-turtlebot3-sbc-image")

Download the correct image file for your hardware and ROS version.  

Kinetic version images use Raspberry Pi OS(Raspbian OS).

[![](/assets/images/icon_download.png) **Download** `Raspberry Pi 3B+` ROS Kinetic image](http://www.robotis.com/service/download.php?no=1738 "http://www.robotis.com/service/download.php?no=1738")

**SHA256** : eb8173f3727db08087990b2c4e2bb211e70bd54644644834771fc8b971856b97

The recovery image files can be modified without a prior notice.

`Raspberry Pi 4` does not support Ubuntu 16.04 nor Debian Jessie, therefore, ROS Kinetic is not supported.

### [Unzip the downloaded image file](#unzip-the-downloaded-image-file "#unzip-the-downloaded-image-file")

Extract the `.img` file and save it in the local disk.

### Burn the image file

You can use various image burning tools.  

For example, `Raspberry Pi Imager` or Linux `Disks` utility can be used.  

Choose your preferred tool to burn the image to microSD.

#### Raspberry Pi Imager

Please refer to [this article](https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/ "https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/") to find more information about Raspberry Pi Imager.

[![](/assets/images/icon_download.png) **Download** Raspberry Pi Imager from raspberrypi.org](https://www.raspberrypi.org/software/ "https://www.raspberrypi.org/software/")

![](/assets/images/platform/turtlebot3/setup/rpi_imager.gif)

1. Click `CHOOSE OS`.
2. Click `Use custom` and select the extracted `.img` file from local disk.
3. Click `CHOOSE STORAGE` and select the microSD.
4. Click `WRITE` to start burning the image.

#### Disks Utility

`Disks` utility is included in recent Ubuntu Desktop. Search for “Disks” and launch the app.

![](/assets/images/platform/turtlebot3/setup/disks.gif)

1. Select the microSD card in the left panel.
2. Select `Restore Disk Image` option.
3. Open the `.img` file from local disk.
4. Click `Start Restoring...` > `Restore` button.

![](/assets/images/icon_unfold.png) **Click to expand : Instructions for Intel Joule 570x**

In case you use Intel Joule 570x, please follow the instructions below.  

Intel Joule is discontinued in 2017, and additional support is unavailable.

1. Download Ubuntu 16.04 image for Intel® Joule™
	* [Download Ubuntu 16.04 for Intel® Joule™](http://people.canonical.com/~platform/snappy/tuchuck/desktop-final/tuchuck-xenial-desktop-iso-20170317-0.iso "http://people.canonical.com/~platform/snappy/tuchuck/desktop-final/tuchuck-xenial-desktop-iso-20170317-0.iso")
2. Create a bootable USB with the downloaded image.
3. Install Ubuntu from the USB

### Boot Up the Raspberry Pi

1. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.
2. Connect input devices to the USB port of Raspberry Pi
3. Insert the microSD card.
4. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.

### Configure the Raspberry Pi

**NOTE** : If you encounter apt failures about the ROS GPG key (due to the existing GPG expiration), you may need to update GPG key. Please see [ROS GPG Key Expiration Incident](#https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669 "#https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669"), and proceed to the given solution.

1. After Raspbian OS is up and running, connect to the WiFi network that is connected with the PC.
2. Find the assigned IP address for Raspberry Pi with the command below. Usually wireless IP address for Raspberry Pi can be found under the `wlan0` section.

```
$ ifconfig

```
3. From your PC, open the terminal and connect to the Raspberry Pi with its IP address.  

 The default password is set as **turtlebot**.

```
$ ssh pi@{IP_ADDRESS_OF_RASPBERRY_PI}

```
4. Once logged in to the Raspberry Pi, execute the commands below to sync time.

```
$ sudo apt-get install ntpdate
$ sudo ntpdate ntp.ubuntu.com

```
5. Load Raspberry Pi configuration interface.

```
$ sudo raspi-config

```
6. Select `Advanced Options` > `Expand Filesystem` and exit.
7. Network configuration for ROS

```
$ nano ~/.bashrc

```
8. Go to the end of file with `Ctrl`+`END` or `Alt`+`/`, then modify the IP adddresses of `ROS_MASTER_URI` and the `ROS_HOSTNAME`.

```
export ROS\_MASTER\_URI=http://{IP_ADDRESS_OF_REMOTE_PC}:11311
export ROS\_HOSTNAME={IP_ADDRESS_OF_RASPBERRY_PI_3}

```
9. Save the file and exit the nano editor.
10. Apply changes with the command below.

```
$ source ~/.bashrc

```

### NEW LDS-02 Configuration

The TurtleBot3 LDS has been updated to LDS-02 since 2022 models.  

Please follow the instructions below on the **SBC (Raspberry Pi)** of TurtleBot3.

| LDS-01 | LDS-02 |
| --- | --- |
|  |  |

1. Install the LDS-02 driver and update TurtleBot3 package.

```
$ sudo apt update
$ sudo apt install libudev-dev
$ cd ~/catkin_ws/src
$ git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/catkin_ws/src/turtlebot3 && git pull
$ rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
$ cd ~/catkin_ws && catkin_make

```
2. Export the LDS\_MODEL to the bashrc file. Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-02' >> ~/.bashrc

```
3. Apply changes with the command below.

```
$ source ~/.bashrc

```

## [SBC Setup](#sbc-setup "#sbc-setup")

**WARNING**

* This process may take long time. Please do not use battery while following this section.
* An HDMI monitor and input devices such as a keyboard and a mouse will be required to complete this setup.
* In order to use the webOS Robotics Platform, please refer to [webOS Robotics Platform](https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions "https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions") instruction. Packages will be cross-compiled using OpenEmbedded on a higher performance PC and an image file is created.

TurtleBot3 Hardware is compatible with Jetson Nano SBC.  

Please refer to the video below in order to set up the Jetson Nano for TurtleBot3.  

The [Jetson Nano Developer Kit setup](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit "https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit") must be completed first.

### [Prepare microSD Card and Reader](#prepare-microsd-card-and-reader "#prepare-microsd-card-and-reader")

If you PC do not have a microSD slot, please use a microSD card reader to burn the recovery image.  

![](/assets/images/platform/turtlebot3/setup/micro_sd_reader.png)

### [Download TurtleBot3 SBC Image](#download-turtlebot3-sbc-image "#download-turtlebot3-sbc-image")

Download the correct image file for your hardware and ROS version.  

Melodic version images are created based on Ubuntu 18.04.

[![](/assets/images/icon_download.png) **Download** `Raspberry Pi 3B+` ROS Melodic image](https://www.robotis.com/service/download.php?no=2011 "https://www.robotis.com/service/download.php?no=2011")

**SHA256** : 312e1a5ad78447b901ae401ba31b2aaf94c1c760bdcafc60e2312df14e342640

[![](/assets/images/icon_download.png) **Download** `Raspberry Pi 4B (2GB or 4GB)` ROS Melodic image](https://www.robotis.com/service/download.php?no=2065 "https://www.robotis.com/service/download.php?no=2065")

**SHA256** : 676bbcfc27fc6990bdf1e026247008f0525d344ccfaa106dca6c53d0bf7f4de8

* Please note that this image may not compatible with Raspberry Pi 4B with 8GB RAM.

[![](/assets/images/icon_download.png) **Download** `Raspberry Pi 4B (2GB or 4GB)` ROS Melodic image(Raspberry Pi OS based)](https://www.robotis.com/service/download.php?no=1905 "https://www.robotis.com/service/download.php?no=1905")

**SHA256** : 73546c63d3056bfc5538acc187f54dab6c1601096df320e60e0842bcb1b03d34

* ROS Melodic recovery image based on Ubuntu 18.04 above is recommended.
* Please note that this image may not compatible with Raspberry Pi 4B with 8GB RAM.

The recovery image files can be modified without a prior notice.

### [Unzip the downloaded image file](#unzip-the-downloaded-image-file "#unzip-the-downloaded-image-file")

Extract the `.img` file and save it in the local disk.

### Burn the image file

You can use various image burning tools.  

For example, `Raspberry Pi Imager` or Linux `Disks` utility can be used.  

Choose your preferred tool to burn the image to microSD.

#### Raspberry Pi Imager

Please refer to [this article](https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/ "https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/") to find more information about Raspberry Pi Imager.

[![](/assets/images/icon_download.png) **Download** Raspberry Pi Imager from raspberrypi.org](https://www.raspberrypi.org/software/ "https://www.raspberrypi.org/software/")

![](/assets/images/platform/turtlebot3/setup/rpi_imager.gif)

1. Click `CHOOSE OS`.
2. Click `Use custom` and select the extracted `.img` file from local disk.
3. Click `CHOOSE STORAGE` and select the microSD.
4. Click `WRITE` to start burning the image.

#### Disks Utility

`Disks` utility is included in the recent Ubuntu Desktop. Search for “Disks” and launch the app.

![](/assets/images/platform/turtlebot3/setup/disks.gif)

1. Select the microSD card in the left panel.
2. Select `Restore Disk Image` option.
3. Open the `.img` file from local disk.
4. Click `Start Restoring...` > `Restore` button.

### Resize the Partition

In order to reduce the size of recovery image file and to decrease the time to burn the image onto microSD, the recovery partition is minimized.  

Please resize the partition to use the unallocated space.

**Be aware of selecting an incorrect disk or a partition. Partitioning a system disk of your PC may cause a serious system malfunction.**

[![](/assets/images/icon_download.png) Download or install GParted GUI tool](https://gparted.org/download.php "https://gparted.org/download.php")

![](/assets/images/platform/turtlebot3/setup/gparted.gif)

1. Select microSD card from the menu (mounted location may vary by system).
2. Right click on the yellow partition.
3. Select `Resize/Move` option.
4. Drag the right edge of the partition to all the way to the right end.
5. Click `Resize/Move` button.
6. Click the `Apply All Operations` green check button at the top.

### Configure the WiFi Network Setting

**NOTE** : If you encounter apt failures about the ROS GPG key (due to the existing GPG expiration), you may need to update GPG key. Please see [ROS GPG Key Expiration Incident](#https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669 "#https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669"), and proceed to the given solution.

1. Open a terminal window with `Alt`+`Ctrl`+`T` and go to the netplan directory in the microSD card.  

Start editing the `50-cloud-init.yaml` file with a superuser permission `sudo`.

```
$ cd /media/$USER/writable/etc/netplan
$ sudo nano 50-cloud-init.yaml

```

When the editor is opened, replace the `WIFI_SSID` and `WIFI_PASSWORD` with your wifi SSID and password.  

![](/assets/images/platform/turtlebot3/setup/ros2_sbc_netcfg.png)  

Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.  

![](/assets/images/platform/turtlebot3/setup/network_setup.gif)

If “No such file or directory” is returned, make sure the microSD is mounted to the system.

1. Boot Up the Raspberry Pi  

 a. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.  

 b. Connect input devices to the USB port of Raspberry Pi.  

 c. Insert the microSD card.  

 d. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.  

 e. Login with ID `ubuntu` and PASSWORD `turtlebot`.

HDMI cable has to be connected before powering the Raspberry Pi, or else the HDMI port of the Raspberry Pi will be disabled.

### ROS Network Configuration

Please follow the instructions below on the **SBC (Raspberry Pi)**.

1. Confirm the WiFi IP address.

```
$ ifconfig

```
2. Edit the `.bashrc` file.

```
$ nano ~/.bashrc

```
3. Find the `ROS_MASTER_URI` and `ROS_HOSTNAME` setting section, then modify the IP adddresses accordingly.

```
export ROS\_MASTER\_URI=http://{IP_ADDRESS_OF_REMOTE_PC}:11311
export ROS\_HOSTNAME={IP_ADDRESS_OF_RASPBERRY_PI_3}

```
4. Save the file with `Ctrl` + `S` and exit the nano editor with `Ctrl` + `X`.
5. Apply changes with the command below.

```
$ source ~/.bashrc

```

![](/assets/images/platform/turtlebot3/setup/ros1_sbc_netcfg.gif)

### NEW LDS-02 Configuration

| LDS-01 | LDS-02 |
| --- | --- |
|  |  |

The TurtleBot3 LDS has been updated to LDS-02 since 2022 models.  

Please follow the instructions below on the **SBC (Raspberry Pi)** of TurtleBot3.

1. Install the LDS-02 driver and update TurtleBot3 package

```
$ sudo apt update
$ sudo apt install libudev-dev
$ cd ~/catkin_ws/src
$ git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/catkin_ws/src/turtlebot3 && git pull
$ rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
$ cd ~/catkin_ws && catkin_make

```
2. Export the LDS\_MODEL to the bashrc file. Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-02' >> ~/.bashrc
$ source ~/.bashrc

```

**This is it! Now you are done with SBC setup :)**  

Next Step : [OpenCR Setup](/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup "/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup")

![](/assets/images/icon_unfold.png) **Click to expand : Manual SBC Setup Instructions**

Please be aware that this manual setup takes a lot more time than burning the recovery image file, but allows flexible choice of package installation. **This instruction is not recommended for the beginners**.

1. ![](/assets/images/icon_download.png) Download the `ubuntu-18.04.4-preinstalled-server-arm64+raspi3.img.xz` image on your PC.
	* [Ubuntu 18.04.4 Preinstalled Server ARM64 for Raspberry Pi3](http://old-releases.ubuntu.com/releases/18.04.4/ "http://old-releases.ubuntu.com/releases/18.04.4/")
2. Extract the downloaded file.
3. Burn the `.img` file to the microSD card. You can use various image burning tools.  

 For example, `Raspberry Pi Imager` or Linux `Disks` utility can be used. Choose your preferred tool to burn the image to microSD.  

![](/assets/images/platform/turtlebot3/setup/rpi_imager.gif)  

 a. Click `CHOOSE OS`.  

 b. Click `Use custom` and select the extracted `.img` file from local disk.  

 c. Click `CHOOSE STORAGE` and select the microSD.  

 d. Click `WRITE` to start burning the image.
4. Boot Up the Raspberry Pi  

 a. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.  

 b. Connect input devices to the USB port of Raspberry Pi.  

 c. Insert the microSD card.  

 d. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.
5. Configure the Raspberry Pi  

 a. Log in with default username(`ubuntu`) and password(`ubuntu`). After logged in, system will ask you to change the password.  

 b. Open automatic update setting file.

```
$ sudo nano /etc/apt/apt.conf.d/20auto-upgrades

```
6. Change the update settings as below.

```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";

```

a. Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
7. Enter below command to configure the WiFi network setting.

```
$ sudo nano /etc/netplan/50-cloud-init.yaml

```
8. When the editor is opened, append below contents at the end of the file.  

 Replace the `WIFI_SSID` and `WIFI_PASSWORD` with your wifi SSID and password.  

![](/assets/images/platform/turtlebot3/setup/ros2_sbc_netcfg.png)  

 Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
9. Reboot the Raspberry Pi.

```
$ sudo reboot

```
10. Set the `systemd` to prevent boot-up delay even if there is no network at startup. Run the command below to set mask the `systemd` process using the following command.

```
$ systemctl mask systemd-networkd-wait-online.service

```
11. Disable Suspend and Hibernation

```
$ sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

```
12. After rebooting the Raspberry Pi, if you wish to work from the Remote PC using SSH, use below command from the remote PC terminal. The default password is **ubuntu**.

```
$ ssh ubuntu@{IP Address of Raspberry PI}

```
13. Install ROS Melodic Morenia
Enter below commands to the terminal one at a time.  

In order to check the details of the easy installation script, please refer to [the script file](https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic_rpi.sh "https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic_rpi.sh").

```
$ sudo apt-get update
$ sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic_rpi.sh
$ chmod 755 ./install_ros_melodic_rpi.sh
$ bash ./install_ros_melodic_rpi.sh

```

If the above installation fails, please refer to [the official ROS Melodic installation guide](http://wiki.ros.org/melodic/Installation/Ubuntu "http://wiki.ros.org/melodic/Installation/Ubuntu").
14. Install and Build ROS Packages.

```
$ sudo apt install ros-melodic-rosserial-python ros-melodic-tf
$ mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
$ sudo apt install ros-melodic-hls-lfcd-lds-driver
$ sudo apt install ros-melodic-turtlebot3-msgs
$ sudo apt install ros-melodic-dynamixel-sdk
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws/src/turtlebot3
$ rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
$ cd ~/catkin_ws/
$ echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc
$ source ~/.bashrc
$ cd ~/catkin_ws && catkin_make -j1
$ echo 'source ~/catkin\_ws/devel/setup.bash' >> ~/.bashrc
$ source ~/.bashrc

```
15. USB Port Setting

```
$ rosrun turtlebot3_bringup create_udev_rules

```
16. ROS Network Configuration
Confirm the WiFi IP address and edit the `.bashrc` file

```
$ nano ~/.bashrc

```
17. Modify the IP adddresses of `ROS_MASTER_URI` and the `ROS_HOSTNAME`.

```
export ROS\_MASTER\_URI=http://{IP_ADDRESS_OF_REMOTE_PC}:11311
export ROS\_HOSTNAME={IP_ADDRESS_OF_RASPBERRY_PI_3}

```
18. Save the file and exit the nano editor.
19. LDS Configuration
The TurtleBot3 LDS has been updated to LDS-02 since 2022 models.  

Please follow the instructions below on the **SBC (Raspberry Pi)** of TurtleBot3.

```
$ sudo apt update
$ sudo apt install libudev-dev
$ cd ~/catkin_ws/src
$ git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/catkin_ws && catkin_make

```
20. Export the LDS\_MODEL to the bashrc file. Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-01' >> ~/.bashrc
$ source ~/.bashrc

```
21. Apply changes with the command below.

```
$ source ~/.bashrc

```

Please refer to the Ubuntu Blog below for more useful information.

* [Improving Security with Ubuntu](https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu "https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu")
* [Improving User Experience of TurtleBot3 Waffle Pi](https://ubuntu.com/blog/building-a-better-turtlebot3 "https://ubuntu.com/blog/building-a-better-turtlebot3")
* [How to set up TurtleBot3 Waffle Pi in minutes with Snaps](https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps "https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps")

## [SBC Setup](#sbc-setup "#sbc-setup")

**WARNING**

* This process may take long time. Please do not use battery while following this section.
* An HDMI monitor and input devices such as a keyboard and a mouse will be required to complete this setup.

### [Prepare microSD Card and Reader](#prepare-microsd-card-and-reader "#prepare-microsd-card-and-reader")

If you PC do not have a microSD slot, please use a microSD card reader to burn the recovery image.  

![](/assets/images/platform/turtlebot3/setup/micro_sd_reader.png)

### [Download TurtleBot3 SBC Image](#download-turtlebot3-sbc-image "#download-turtlebot3-sbc-image")

Download the correct image file for your hardware and ROS version.  

Noetic version images are created based on Ubuntu 20.04.

[![](/assets/images/icon_download.png) **Download** `Raspberry Pi 3B+` ROS Noetic image](https://www.robotis.com/service/download.php?no=2008 "https://www.robotis.com/service/download.php?no=2008")

**SHA256** : a7c57e20f2ee4204c95315866f4a274886094f7c63ed390b6d06d95074830309

[![](/assets/images/icon_download.png) **Download** `Raspberry Pi 4B (2GB or 4GB)` ROS Noetic image](https://www.robotis.com/service/download.php?no=2066 "https://www.robotis.com/service/download.php?no=2066")

**SHA256** : 9d48925a78381885916a6f3bb77891adbfae2b271b05fe2ae9a9b7ebd12c46cc

* Please note that this image may not compatible with Raspberry Pi 4B with 8GB RAM.

The recovery image files can be modified without a prior notice.

### [Unzip the downloaded image file](#unzip-the-downloaded-image-file "#unzip-the-downloaded-image-file")

Extract the `.img` file and save it in the local disk.

### Burn the image file

You can use various image burning tools.  

For example, `Raspberry Pi Imager` or Linux `Disks` utility can be used.  

Choose your preferred tool to burn the image to microSD.

#### Raspberry Pi Imager

Please refer to [this article](https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/ "https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/") to find more information about Raspberry Pi Imager.

[![](/assets/images/icon_download.png) **Download** Raspberry Pi Imager from raspberrypi.org](https://www.raspberrypi.org/software/ "https://www.raspberrypi.org/software/")

![](/assets/images/platform/turtlebot3/setup/rpi_imager.gif)

1. Click `CHOOSE OS`.
2. Click `Use custom` and select the extracted `.img` file from local disk.
3. Click `CHOOSE STORAGE` and select the microSD.
4. Click `WRITE` to start burning the image.

#### Disks Utility

`Disks` utility is included in the recent Ubuntu Desktop. Search for “Disks” and launch the app.

![](/assets/images/platform/turtlebot3/setup/disks.gif)

1. Select the microSD card in the left panel.
2. Select `Restore Disk Image` option.
3. Open the `.img` file from local disk.
4. Click `Start Restoring...` > `Restore` button.

### Resize the Partition

In order to reduce the size of recovery image file and to decrease the time to burn the image onto microSD, the recovery partition is minimized.  

Please resize the partition to use the unallocated space.

**Be aware of selecting an incorrect disk or a partition. Partitioning a system disk of your PC may cause a serious system malfunction.**

[![](/assets/images/icon_download.png) Download or install GParted GUI tool](https://gparted.org/download.php "https://gparted.org/download.php")

![](/assets/images/platform/turtlebot3/setup/gparted.gif)

1. Select microSD card from the menu (mounted location may vary by system).
2. Right click on the yellow partition.
3. Select `Resize/Move` option.
4. Drag the right edge of the partition to all the way to the right end.
5. Click `Resize/Move` button.
6. Click the `Apply All Operations` green check button at the top.

### Configure the WiFi Network Setting

1. Open a terminal window with `Alt`+`Ctrl`+`T` and go to the netplan directory in the microSD card.  

Start editing the `50-cloud-init.yaml` file with a superuser permission `sudo`.

```
$ cd /media/$USER/writable/etc/netplan
$ sudo nano 50-cloud-init.yaml

```

When the editor is opened, replace the `WIFI_SSID` and `WIFI_PASSWORD` with your wifi SSID and password.  

![](/assets/images/platform/turtlebot3/setup/ros2_sbc_netcfg.png)  

Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.  

![](/assets/images/platform/turtlebot3/setup/network_setup.gif)

If “No such file or directory” is returned, make sure the microSD is mounted to the system.

1. Boot Up the Raspberry Pi  

 a. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.  

 b. Connect input devices to the USB port of Raspberry Pi.  

 c. Insert the microSD card.  

 d. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.  

 e. Login with ID `ubuntu` and PASSWORD `turtlebot`.

HDMI cable has to be connected before powering the Raspberry Pi, or else the HDMI port of the Raspberry Pi will be disabled.

### ROS Network Configuration

**NOTE** : If you encounter apt failures about the ROS GPG key (due to the existing GPG expiration), you may need to update GPG key. Please see [ROS GPG Key Expiration Incident](#https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669 "#https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669"), and proceed to the given solution.

Please follow the instructions below on the **SBC (Raspberry Pi)**.

1. Confirm the WiFi IP address.

```
$ ifconfig

```
2. Edit the `.bashrc` file.

```
$ nano ~/.bashrc

```
3. Find the `ROS_MASTER_URI` and `ROS_HOSTNAME` setting section, then modify the IP adddresses accordingly.

```
export ROS\_MASTER\_URI=http://{IP_ADDRESS_OF_REMOTE_PC}:11311
export ROS\_HOSTNAME={IP_ADDRESS_OF_RASPBERRY_PI_3}

```
4. Save the file with `Ctrl` + `S` and exit the nano editor with `Ctrl` + `X`.
5. Apply changes with the command below.

```
$ source ~/.bashrc

```

![](/assets/images/platform/turtlebot3/setup/ros1_sbc_netcfg.gif)

### NEW LDS-02 Configuration

| LDS-01 | LDS-02 |
| --- | --- |
|  |  |

The TurtleBot3 LDS has been updated to LDS-02 since 2022 models.  

Please follow the instructions below on the **SBC (Raspberry Pi)** of TurtleBot3.

1. Install the LDS-02 driver and update TurtleBot3 package

```
$ sudo apt update
$ sudo apt install libudev-dev
$ cd ~/catkin_ws/src
$ git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/catkin_ws/src/turtlebot3 && git pull
$ rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
$ cd ~/catkin_ws && catkin_make

```
2. Export the LDS\_MODEL to the bashrc file. Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-02' >> ~/.bashrc
$ source ~/.bashrc

```

**This is it! Now you are done with SBC setup :)**  

Next Step : [OpenCR Setup](/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup "/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup")

![](/assets/images/icon_unfold.png) **Click to expand : Manual SBC Setup Instructions**

Please be aware that this manual setup takes a lot more time than burning the recovery image file, but allows flexible choice of package installation. **This instruction is not recommended for the beginners**.

1. ![](/assets/images/icon_download.png) Download the proper `Ubuntu 20.04.1(Focal) Preinstalled Server` image on your PC.
	* [Ubuntu 20.04.1(Focal) Preinstalled Server for Raspberry Pi3(arm64)](http://cdimage.ubuntu.com/ubuntu-server/focal/daily-preinstalled/current/ "http://cdimage.ubuntu.com/ubuntu-server/focal/daily-preinstalled/current/")
2. Extract the downloaded file.
3. Burn the `.img` file to the microSD card. You can use various image burning tools.  

 For example, `Raspberry Pi Imager` or Linux `Disks` utility can be used. Choose your preferred tool to burn the image to microSD.  

![](/assets/images/platform/turtlebot3/setup/rpi_imager.gif)  

 a. Click `CHOOSE OS`.  

 b. Click `Use custom` and select the extracted `.img` file from local disk.  

 c. Click `CHOOSE STORAGE` and select the microSD.  

 d. Click `WRITE` to start burning the image.
4. Boot Up the Raspberry Pi  

 a. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.  

 b. Connect input devices to the USB port of Raspberry Pi.  

 c. Insert the microSD card.  

 d. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.
5. Configure the Raspberry Pi  

 a. Log in with default username(`ubuntu`) and password(`ubuntu`). After logged in, system will ask you to change the password.  

 b. Open automatic update setting file.

```
$ sudo nano /etc/apt/apt.conf.d/20auto-upgrades

```
6. Change the update settings as below.

```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";

```

a. Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
7. Enter below command to configure the WiFi network setting.

```
$ sudo nano /etc/netplan/50-cloud-init.yaml

```
8. When the editor is opened, append below contents at the end of the file.  

 Replace the `WIFI_SSID` and `WIFI_PASSWORD` with your wifi SSID and password.  

![](/assets/images/platform/turtlebot3/setup/ros2_sbc_netcfg.png)  

 Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
9. Reboot the Raspberry Pi.

```
$ sudo reboot

```
10. Set the `systemd` to prevent boot-up delay even if there is no network at startup. Run the command below to set mask the `systemd` process using the following command.

```
$ systemctl mask systemd-networkd-wait-online.service

```
11. Disable Suspend and Hibernation

```
$ sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

```
12. After rebooting the Raspberry Pi, if you wish to work from the Remote PC using SSH, use below command from the remote PC terminal. The default password is **ubuntu**.

```
$ ssh ubuntu@{IP Address of Raspberry PI}

```
13. Install ROS Noetic Ninjemys
Enter below commands to the terminal one at a time.  

In order to check the details of the easy installation script, please refer to [the script file](https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic_rpi.sh "https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic_rpi.sh").

```
$ sudo apt-get update
$ sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic_rpi.sh
$ chmod 755 ./install_ros_noetic_rpi.sh
$ bash ./install_ros_noetic_rpi.sh

```

If the above installation fails, please refer to [the official ROS Noetic installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu "http://wiki.ros.org/noetic/Installation/Ubuntu").
14. Install and Build ROS Packages.

```
$ sudo apt install ros-noetic-rosserial-python ros-noetic-tf
$ sudo apt install ros-noetic-hls-lfcd-lds-driver
$ sudo apt install ros-noetic-turtlebot3-msgs
$ sudo apt install ros-noetic-dynamixel-sdk
$ cd ~/catkin_ws/src
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws/src/turtlebot3
$ rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
$ cd ~/catkin_ws/
$ catkin_make -j1
$ source ~/.bashrc

```
15. USB Port Setting

```
$ rosrun turtlebot3_bringup create_udev_rules

```
16. ROS Network Configuration
Confirm the WiFi IP address and edit the `.bashrc` file

```
$ nano ~/.bashrc

```
17. Modify the IP adddresses of `ROS_MASTER_URI` and the `ROS_HOSTNAME`.

```
export ROS\_MASTER\_URI=http://{IP_ADDRESS_OF_REMOTE_PC}:11311
export ROS\_HOSTNAME={IP_ADDRESS_OF_RASPBERRY_PI_3}

```
18. Save the file and exit the nano editor.
19. LDS Configuration
The TurtleBot3 LDS has been updated to LDS-02 since 2022 models.  

Please follow the instructions below on the **SBC (Raspberry Pi)** of TurtleBot3.

```
$ sudo apt update
$ sudo apt install libudev-dev
$ cd ~/catkin_ws/src
$ git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/catkin_ws && catkin_make

```
20. Export the LDS\_MODEL to the bashrc file. Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-01' >> ~/.bashrc
$ source ~/.bashrc

```
21. Apply changes with the command below.

```
$ source ~/.bashrc

```

Please refer to the Ubuntu Blog below for more useful information.

* [Improving Security with Ubuntu](https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu "https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu")
* [Improving User Experience of TurtleBot3 Waffle Pi](https://ubuntu.com/blog/building-a-better-turtlebot3 "https://ubuntu.com/blog/building-a-better-turtlebot3")
* [How to set up TurtleBot3 Waffle Pi in minutes with Snaps](https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps "https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps")

## [SBC Setup](#sbc-setup "#sbc-setup")

**WARNING**

* This SBC Setup section is specifically written for **Raspberry Pi 3B+** which is the current official TurtleBot3 SBC.
* This process may take long time. Please do not use battery while following this section.
* An HDMI monitor and input devices such as a keyboard and a mouse will be required to complete this setup.
* In order to use the webOS Robotics Platform, please refer to [webOS Robotics Platform](https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions "https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions") instruction. Packages will be cross-compiled using OpenEmbedded on a higher performance PC and an image file is created.

### [Prepare microSD Card and Reader](#prepare-microsd-card-and-reader "#prepare-microsd-card-and-reader")

If you PC do not have a microSD slot, please use a microSD card reader to burn the recovery image.  

![](/assets/images/platform/turtlebot3/setup/micro_sd_reader.png)

### [Download SBC OS Image](#download-sbc-os-image "#download-sbc-os-image")

Download the correct image file for your hardware and ROS version.  

ROS2 Dashing requires Ubuntu 18.04.

[![](/assets/images/icon_download.png) **Download for Rasbperry Pi 4** `ubuntu-18.04.4-preinstalled-server-arm64+raspi4.img.xz` OS image](http://old-releases.ubuntu.com/releases/18.04.4/ "http://old-releases.ubuntu.com/releases/18.04.4/")

[![](/assets/images/icon_download.png) **Download for Raspberry Pi 3B+** `ubuntu-18.04.3-preinstalled-server-arm64+raspi3.img.xz` OS image](http://old-releases.ubuntu.com/releases/18.04.3/ "http://old-releases.ubuntu.com/releases/18.04.3/")

### [Unzip the downloaded image file](#unzip-the-downloaded-image-file "#unzip-the-downloaded-image-file")

Extract the `.img` file and save it in the local disk.

### Burn the image file

You can use various image burning tools.  

For example, `Raspberry Pi Imager` or Linux `Disks` utility can be used.  

Choose your preferred tool to burn the image to microSD.

#### Raspberry Pi Imager

Please refer to [this article](https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/ "https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/") to find more information about Raspberry Pi Imager.

[![](/assets/images/icon_download.png) **Download** Raspberry Pi Imager from raspberrypi.org](https://www.raspberrypi.org/software/ "https://www.raspberrypi.org/software/")

![](/assets/images/platform/turtlebot3/setup/rpi_imager.gif)

1. Click `CHOOSE OS`.
2. Click `Use custom` and select the extracted `.img` file from local disk.
3. Click `CHOOSE STORAGE` and select the microSD.
4. Click `WRITE` to start burning the image.

#### Disks Utility

`Disks` utility is included in the recent Ubuntu Desktop. Search for “Disks” and launch the app.

![](/assets/images/platform/turtlebot3/setup/disks.gif)

1. Select the microSD card in the left panel.
2. Select `Restore Disk Image` option.
3. Open the `.img` file from local disk.
4. Click `Start Restoring...` > `Restore` button.

### Boot Up the Raspberry Pi

1. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.
2. Connect input devices to the USB port of Raspberry Pi
3. Insert the microSD card.
4. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.
5. Login with ID `ubuntu` and PASSWORD `ubuntu`.  

 Once logged in, the system will request to change the password.

HDMI cable has to be connected before powering the Raspberry Pi, or else the HDMI port of the Raspberry Pi will be disabled.

### Configure the Raspberry Pi

1. Open automatic update setting file.

```
$ sudo nano /etc/apt/apt.conf.d/20auto-upgrades

```
2. Edit to disable automatic update settings as below.

```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";

```

Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
3. Enter below command to configure the WiFi network setting.

```
$ sudo nano /etc/netplan/50-cloud-init.yaml

```
4. When the editor is opened, append below contents at the end of the file.  

 Replace the `WIFI_SSID` and `WIFI_PASSWORD` with your wifi SSID and password.  

![](/assets/images/platform/turtlebot3/setup/ros2_sbc_netcfg.png)  

 Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
5. Apply all configuration for the renderers, and then reboot the Raspberry Pi.

```
$ sudo netplan apply
$ reboot

```
6. Set the `systemd` to prevent boot-up delay even if there is no network at startup. Run the command below to set mask the `systemd` process using the following command.

```
$ systemctl mask systemd-networkd-wait-online.service

```
7. Disable Suspend and Hibernation

```
$ sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

```
8. Install and enable the SSH

```
$ sudo apt install ssh
$ sudo systemctl enable --now ssh
$ reboot

```
9. After rebooting the Raspberry Pi, if you wish to work from the Remote PC using SSH, use below command from the remote PC terminal. The default password is **ubuntu**.

```
$ ssh ubuntu@{IP Address of Raspberry PI}

```

### [Add Swap Space](#add-swap-space "#add-swap-space")

1. Enter the command below to create 2GB swap space.

```
$ sudo swapoff /swapfile
$ sudo fallocate -l 2G /swapfile
$ sudo chmod 600 /swapfile
$ sudo mkswap /swapfile
$ sudo swapon /swapfile
$ sudo nano /etc/fstab

```

You can ignore below error when entering `swapoff /swapfile` command.

```
swapoff: /swapfile: swapoff failed: No such file or directory

```
2. When the editor opens the fstab file, append below contents at the end of the file.

```
/swapfile swap swap defaults 0 0

```
3. Check if 2GB of swap space is correctly configured.

```
$ sudo free -h
              total        used        free      shared  buff/cache   available
Mem:           912M         97M        263M        4.4M        550M        795M
Swap:          2.0G          0B        2.0G

```

Please refer to the Ubuntu Blog below for more useful information.

* [Improving Security with Ubuntu](https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu "https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu")
* [Improving User Experience of TurtleBot3 Waffle Pi](https://ubuntu.com/blog/building-a-better-turtlebot3 "https://ubuntu.com/blog/building-a-better-turtlebot3")
* [How to set up TurtleBot3 Waffle Pi in minutes with Snaps](https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps "https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps")

### Install ROS 2 Dashing Diademata

**Reference**: [Official ROS 2 Dashing Installation Guide](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/ "https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/")

1. Open a terminal on **SBC**
2. Setup locale

```
$ sudo apt update && sudo apt install locales
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC\_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8

```
3. Setup sources

```
$ sudo apt update && sudo apt install curl gnupg2 lsb-release
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

```
4. Install ROS 2 packages

```
$ sudo apt update
$ sudo apt install ros-dashing-ros-base

```
5. Install and Build ROS Packages.

```
$ sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
$ sudo apt install ros-dashing-hls-lfcd-lds-driver
$ sudo apt install ros-dashing-turtlebot3-msgs
$ sudo apt install ros-dashing-dynamixel-sdk
$ mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
$ git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/turtlebot3_ws/src/turtlebot3
$ rm -r turtlebot3_cartographer turtlebot3_navigation2
$ cd ~/turtlebot3_ws/
$ echo 'source /opt/ros/dashing/setup.bash' >> ~/.bashrc
$ source ~/.bashrc
$ colcon build --symlink-install --parallel-workers 1
$ echo 'source ~/turtlebot3\_ws/install/setup.bash' >> ~/.bashrc
$ source ~/.bashrc

```
6. USB Port Setting for OpenCR

```
$ sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger

```
7. ROS Domain ID Setting
In ROS2 DDS communication, `ROS_DOMAIN_ID` must be matched between **Remote PC** and **TurtleBot3** for communication under the same network environment. Following commands shows how to assign a `ROS_DOMAIN_ID` to SBC in TurtleBot3.
	* A default ID of **TurtleBot3** is `30`.
	* Configuring the `ROS_DOMAIN_ID` of Remote PC and SBC in TurtleBot3 to `30` is recommended.

```
$ echo 'export ROS\_DOMAIN\_ID=30 #TURTLEBOT3' >> ~/.bashrc
$ source ~/.bashrc

```

**WARNING** : Do not use an identical ROS\_DOMAIN\_ID with others in the same network. It will cause a conflict of communication between users under the same network environment.

### NEW LDS-02 Configuration

| LDS-01 | LDS-02 |
| --- | --- |
|  |  |

The TurtleBot3 LDS has been updated to LDS-02 since 2022 models.  

Please follow the instructions below on the **SBC (Raspberry Pi)** of TurtleBot3.

1. Install the LDS-02 driver and update TurtleBot3 package

```
$ sudo apt update
$ sudo apt install libudev-dev
$ cd ~/turtlebot3_ws/src
$ git clone -b dashing-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/turtlebot3_ws && colcon build --symlink-install

```
2. Export the LDS\_MODEL to the bashrc file. Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-02' >> ~/.bashrc
$ source ~/.bashrc

```

## [SBC Setup](#sbc-setup "#sbc-setup")

**WARNING**

* This process may take long time. Please do not use battery while following this section.
* An HDMI monitor and input devices such as a keyboard and a mouse will be required to complete this setup.
* In order to use the webOS Robotics Platform, please refer to [webOS Robotics Platform](https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions "https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions") instruction. Packages will be cross-compiled using OpenEmbedded on a higher performance PC and an image file is created.

### [Prepare microSD Card and Reader](#prepare-microsd-card-and-reader "#prepare-microsd-card-and-reader")

If you PC do not have a microSD slot, please use a microSD card reader to burn the recovery image.  

![](/assets/images/platform/turtlebot3/setup/micro_sd_reader.png)

### [Download TurtleBot3 SBC Image](#download-turtlebot3-sbc-image "#download-turtlebot3-sbc-image")

Download the correct image file for your hardware and ROS version.  

Foxy version images are created based on Ubuntu 20.04.

[![](/assets/images/icon_download.png) **Download** `Raspberry Pi 3B+` ROS2 Foxy image](https://www.robotis.com/service/download.php?no=2058 "https://www.robotis.com/service/download.php?no=2058")

**SHA256** : e1916b75573e3944c72552664ee1e32e9be32a026bd5b4323d0a4b5778243a1e

[![](/assets/images/icon_download.png) **Download** `Raspberry Pi 4B (2GB or 4GB)` ROS2 Foxy image](https://www.robotis.com/service/download.php?no=2064 "https://www.robotis.com/service/download.php?no=2064")

**SHA256** : 8b8b54ad80c7a02ae35da8e9e5d9750fdf21ec6098052a804986ab22ce10ba7e

* Please note that this image may not compatible with Raspberry Pi 4B with 8GB RAM.

The recovery image files can be modified without a prior notice.

### [Unzip the downloaded image file](#unzip-the-downloaded-image-file "#unzip-the-downloaded-image-file")

Extract the `.img` file and save it in the local disk.

### Burn the image file

You can use various image burning tools.  

For example, `Raspberry Pi Imager` or Linux `Disks` utility can be used.  

Choose your preferred tool to burn the image to microSD.

#### Raspberry Pi Imager

Please refer to [this article](https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/ "https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/") to find more information about Raspberry Pi Imager.

[![](/assets/images/icon_download.png) **Download** Raspberry Pi Imager from raspberrypi.org](https://www.raspberrypi.org/software/ "https://www.raspberrypi.org/software/")

![](/assets/images/platform/turtlebot3/setup/rpi_imager.gif)

1. Click `CHOOSE OS`.
2. Click `Use custom` and select the extracted `.img` file from local disk.
3. Click `CHOOSE STORAGE` and select the microSD.
4. Click `WRITE` to start burning the image.

#### Disks Utility

`Disks` utility is included in the recent Ubuntu Desktop. Search for “Disks” and launch the app.

![](/assets/images/platform/turtlebot3/setup/disks.gif)

1. Select the microSD card in the left panel.
2. Select `Restore Disk Image` option.
3. Open the `.img` file from local disk.
4. Click `Start Restoring...` > `Restore` button.

### Resize the Partition

In order to reduce the size of recovery image file and to decrease the time to burn the image onto microSD, the recovery partition is minimized.  

Please resize the partition to use the unallocated space.

**Be aware of selecting an incorrect disk or a partition. Partitioning a system disk of your PC may cause a serious system malfunction.**

[![](/assets/images/icon_download.png) Download or install GParted GUI tool](https://gparted.org/download.php "https://gparted.org/download.php")

![](/assets/images/platform/turtlebot3/setup/gparted.gif)

1. Select microSD card from the menu (mounted location may vary by system).
2. Right click on the yellow partition.
3. Select `Resize/Move` option.
4. Drag the right edge of the partition to all the way to the right end.
5. Click `Resize/Move` button.
6. Click the `Apply All Operations` green check button at the top.

### Configure the WiFi Network Setting

**NOTE** : If you encounter apt failures about the ROS GPG key (due to the existing GPG expiration), you may need to update GPG key. Please see [ROS GPG Key Expiration Incident](#https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669 "#https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669"), and proceed to the given solution.

1. Open a terminal window with `Alt`+`Ctrl`+`T` and go to the netplan directory in the microSD card.  

Start editing the `50-cloud-init.yaml` file with a superuser permission `sudo`.

```
$ cd /media/$USER/writable/etc/netplan
$ sudo nano 50-cloud-init.yaml

```

When the editor is opened, replace the `WIFI_SSID` and `WIFI_PASSWORD` with your wifi SSID and password.  

![](/assets/images/platform/turtlebot3/setup/ros2_sbc_netcfg.png)  

Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.  

![](/assets/images/platform/turtlebot3/setup/network_setup.gif)

If “No such file or directory” is returned, make sure the microSD is mounted to the system.

1. Boot Up the Raspberry Pi  

 a. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.  

 b. Connect input devices to the USB port of Raspberry Pi.  

 c. Insert the microSD card.  

 d. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.  

 e. Login with ID `ubuntu` and PASSWORD `turtlebot`.

HDMI cable has to be connected before powering the Raspberry Pi, or else the HDMI port of the Raspberry Pi will be disabled.

### ROS2 Network Configuration

In ROS2 DDS communication, `ROS_DOMAIN_ID` must be matched between **Remote PC** and **TurtleBot3** for communication under the same network environment.  

The default ROS Domain ID for TurtleBot3 is set to `30` in the ***.bashrc*** file.  

Please modify the ID to avoid any conflict when there are identical ID in the same network.

```
ROS_DOMAIN_ID=30 #TURTLEBOT3

```

**WARNING** : Do not use an identical ROS\_DOMAIN\_ID with others in the same network. It will cause a conflict of communication between users under the same network environment.

### NEW LDS-02 Configuration

| LDS-01 | LDS-02 |
| --- | --- |
|  |  |

The TurtleBot3 LDS has been updated to LDS-02 since 2022 models.  

Please follow the instructions below on the **SBC (Raspberry Pi)** of TurtleBot3.

1. Install the LDS-02 driver and update TurtleBot3 package

```
$ sudo apt update
$ sudo apt install libudev-dev
$ cd ~/turtlebot3_ws/src
$ git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/turtlebot3_ws/src/turtlebot3 && git pull
$ rm -r turtlebot3_cartographer turtlebot3_navigation2
$ cd ~/turtlebot3_ws && colcon build --symlink-install

```
2. Export the LDS\_MODEL to the bashrc file. Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-02' >> ~/.bashrc
$ source ~/.bashrc

```

**This is it! Now you are done with SBC setup :)**  

Next Step : [OpenCR Setup](/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup "/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup")

![](/assets/images/icon_unfold.png) **Click to expand : Manual SBC Setup Instructions**

Please be aware that this manual setup takes a lot more time than burning the recovery image file, but allows flexible choice of package installation. **This instruction is not recommended for the beginners**.

1. ![](/assets/images/icon_download.png) Download the latest `Ubuntu 20.04 server` image for your SBC from the link below.
	* [Ubuntu 20.04 Server 64-bit](https://ubuntu.com/download/raspberry-pi "https://ubuntu.com/download/raspberry-pi")
2. Extract the downloaded file.
3. Burn the `.img` file to the microSD card. You can use various image burning tools.  

 For example, `Raspberry Pi Imager` or Linux `Disks` utility can be used. Choose your preferred tool to burn the image to microSD.  

![](/assets/images/platform/turtlebot3/setup/rpi_imager.gif)  

 a. Click `CHOOSE OS`.  

 b. Click `Use custom` and select the extracted `.img` file from local disk.  

 c. Click `CHOOSE STORAGE` and select the microSD.  

 d. Click `WRITE` to start burning the image.
4. Boot Up the Raspberry Pi  

 a. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.  

 b. Connect input devices to the USB port of Raspberry Pi.  

 c. Insert the microSD card.  

 d. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.
5. Configure the Raspberry Pi  

 a. Log in with default username(`ubuntu`) and password(`ubuntu`). After logged in, system will ask you to change the password.  

 b. Open automatic update setting file.

```
$ sudo nano /etc/apt/apt.conf.d/20auto-upgrades

```
6. Change the update settings as below.

```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";

```

a. Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
7. Enter below command to configure the WiFi network setting.

```
$ sudo nano /etc/netplan/50-cloud-init.yaml

```
8. When the editor is opened, enter below contents to the file.  

 Replace the `WIFI_SSID` and `WIFI_PASSWORD` with your wifi SSID and password.  

![](/assets/images/platform/turtlebot3/setup/ros2_sbc_netcfg.png)  

 Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
9. Reboot the Raspberry Pi.

```
$ sudo reboot

```
10. Set the `systemd` to prevent boot-up delay even if there is no network at startup. Run the command below to set mask the `systemd` process using the following command.

```
$ systemctl mask systemd-networkd-wait-online.service

```
11. Disable Suspend and Hibernation

```
$ sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

```
12. After rebooting the Raspberry Pi, if you wish to work from the Remote PC using SSH, use below command from the remote PC terminal. Make sure to use the password you set in `Step 5`.

```
$ ssh ubuntu@{IP Address of Raspberry PI}

```
13. Install ROS2 Foxy Fiztroy  

Enter below commands to the Raspberry Pi terminal one at a time.  

In order to check the details of the easy installation script, please refer to [the script file](https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy_rpi.sh "https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy_rpi.sh").

```
$ sudo apt update
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy_rpi.sh
$ chmod 755 ./install_ros2_foxy_rpi.sh
$ bash ./install_ros2_foxy_rpi.sh

```

If the above installation fails, please refer to [the official ROS2 Foxy installation guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html "https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html").
14. Install and Build ROS Packages.

```
$ sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
$ sudo apt install ros-foxy-hls-lfcd-lds-driver
$ sudo apt install ros-foxy-turtlebot3-msgs
$ sudo apt install ros-foxy-dynamixel-sdk
$ mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
$ git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/turtlebot3_ws/src/turtlebot3
$ rm -r turtlebot3_cartographer turtlebot3_navigation2
$ cd ~/turtlebot3_ws/
$ echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
$ source ~/.bashrc
$ colcon build --symlink-install --parallel-workers 1
$ echo 'source ~/turtlebot3\_ws/install/setup.bash' >> ~/.bashrc
$ source ~/.bashrc

```
15. USB Port Setting for OpenCR

```
$ sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger

```
16. ROS Domain ID Setting
In ROS2 DDS communication, `ROS_DOMAIN_ID` must be matched between **Remote PC** and **TurtleBot3** for communication under the same network environment. Following commands shows how to assign a `ROS_DOMAIN_ID` to SBC in TurtleBot3.
	* A default ID of **TurtleBot3** is `30`.
	* Configuring the `ROS_DOMAIN_ID` of Remote PC and SBC in TurtleBot3 to `30` is recommended.

```
$ echo 'export ROS\_DOMAIN\_ID=30 #TURTLEBOT3' >> ~/.bashrc
$ source ~/.bashrc

```

**WARNING** : Do not use an identical ROS\_DOMAIN\_ID with others in the same network. It will cause a conflict of communication between users under the same network environment.

1. LDS Configuration
The TurtleBot3 LDS has been updated to LDS-02 since 2022 models.  

Please follow the instructions below on the **SBC (Raspberry Pi)** of TurtleBot3.

```
$ sudo apt update
$ sudo apt install libudev-dev
$ cd ~/turtlebot3_ws/src
$ git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/turtlebot3_ws && colcon build --symlink-install

```
2. Export the LDS\_MODEL to the bashrc file. Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-02' >> ~/.bashrc

```
3. Apply changes with the command below.

```
$ source ~/.bashrc

```

Please refer to the Ubuntu Blog below for more useful information.

* [Improving Security with Ubuntu](https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu "https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu")
* [Improving User Experience of TurtleBot3 Waffle Pi](https://ubuntu.com/blog/building-a-better-turtlebot3 "https://ubuntu.com/blog/building-a-better-turtlebot3")
* [How to set up TurtleBot3 Waffle Pi in minutes with Snaps](https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps "https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps")

## [SBC Setup](#sbc-setup "#sbc-setup")

**WARNING**

* This process may take long time. Please do not use battery while following this section.
* An HDMI monitor and input devices such as a keyboard and a mouse will be required to complete this setup.
* In order to use the webOS Robotics Platform, please refer to [webOS Robotics Platform](https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions "https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions") instruction. Packages will be cross-compiled using OpenEmbedded on a higher performance PC and an image file is created.

### [Prepare microSD Card and Reader](#prepare-microsd-card-and-reader "#prepare-microsd-card-and-reader")

If you PC do not have a microSD slot, please use a microSD card reader to burn the recovery image.  

![](/assets/images/platform/turtlebot3/setup/micro_sd_reader.png)

The microSD card reader is not included in the TurtleBot3 package.

### Install Raspberry Pi Imager

Download the `Raspberry Pi Imager` to install Ubuntu Server 22.04 for Raspberry Pi.  

If the Raspberry Pi Imager is already installed, update to the latest version.  

Please refer to [this article](https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/ "https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/") to find more information about Raspberry Pi Imager.

[![](/assets/images/icon_download.png) **Download** Raspberry Pi Imager from raspberrypi.org](https://www.raspberrypi.org/software/ "https://www.raspberrypi.org/software/")

### Install Ubuntu 22.04

1. Run Raspberry Pi Imager
2. Click `CHOOSE OS`.
3. Select `Other gerneral-purpose OS`.
4. Select `Ubuntu`.
5. Select `Ubuntu Server 22.04.5 LTS (64-bit)` that support RPi 3/4/400.
6. Click `CHOOSE STORAGE` and select the micro SD card.
7. Click `WRITE` to install the Ubuntu.

### Configure the Raspberry Pi

HDMI cable must be connected before powering the Raspberry Pi, or else the HDMI port of the Raspberry Pi will be disabled.

1. Boot Up the Raspberry Pi  

 a. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.  

 b. Connect input devices to the USB port of Raspberry Pi.  

 c. Insert the microSD card.  

 d. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.  

 e. Login with ID `ubuntu` and PASSWORD `ubuntu`. Once logged in, you’ll be asked to change the password.
2. Open the network configuration file with the command below.

```
$ sudo nano /writable/etc/netplan/50-cloud-init.yaml

```
3. When the editor is opened, edit the content as below while replacing the `WIFI_SSID` and `WIFI_PASSWORD` with your actual wifi SSID and password.  

![](/assets/images/platform/turtlebot3/setup/ros2_sbc_netcfg.png)
4. Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
5. Enter the command below to edit automatic update setting file.

```
$ sudo nano /etc/apt/apt.conf.d/20auto-upgrades

```
6. Change the update settings as below.

```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";

```
7. Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
8. Set the `systemd` to prevent boot-up delay even if there is no network at startup. Run the command below to set mask the `systemd` process using the following command.

```
$ systemctl mask systemd-networkd-wait-online.service

```
9. Disable Suspend and Hibernation

```
$ sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

```
10. Reboot the Raspberry Pi.

```
$ reboot

```
11. After rebooting the Raspberry Pi, if you wish to work from the Remote PC using SSH, use below command from the remote PC terminal. Make sure to use the password you set in `Step 1`.

```
$ ssh ubuntu@{IP Address of Raspberry PI}

```
12. Install ROS2 Humble Hawksbill  

Follow the instruction in [the official ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html "https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html").
13. Install and Build ROS Packages.  

Building the `turtlebot3` package may take longer than an hour. Please use the SMPS to ensure the system is always powered.

```
$ sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
$ sudo apt install ros-humble-hls-lfcd-lds-driver
$ sudo apt install ros-humble-turtlebot3-msgs
$ sudo apt install ros-humble-dynamixel-sdk
$ sudo apt install libudev-dev
$ mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
$ git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/turtlebot3_ws/src/turtlebot3
$ rm -r turtlebot3_cartographer turtlebot3_navigation2
$ cd ~/turtlebot3_ws/
$ echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
$ source ~/.bashrc
$ colcon build --symlink-install --parallel-workers 1
$ echo 'source ~/turtlebot3\_ws/install/setup.bash' >> ~/.bashrc
$ source ~/.bashrc

```
14. USB Port Setting for OpenCR

```
$ sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger

```
15. ROS Domain ID Setting
In ROS2 DDS communication, `ROS_DOMAIN_ID` must be matched between **Remote PC** and **TurtleBot3** for communication under the same network environment. Following commands shows how to assign a `ROS_DOMAIN_ID` to SBC in TurtleBot3.
	* A default ID of **TurtleBot3** is `30`.
	* Configuring the `ROS_DOMAIN_ID` of Remote PC and SBC in TurtleBot3 to `30` is recommended.

```
$ echo 'export ROS\_DOMAIN\_ID=30 #TURTLEBOT3' >> ~/.bashrc
$ source ~/.bashrc

```

**WARNING** : Do not use an identical ROS\_DOMAIN\_ID with others in the same network. It will cause a conflict of communication between users under the same network environment.

### LDS Configuration

The TurtleBot3 LDS has been updated to LDS-02 since 2022.  

If you have purchased TurtleBot3 after 2022, please use `LDS-02` for the LDS\_MODEL.

| LDS-01 | LDS-02 |
| --- | --- |
|  |  |

Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-02' >> ~/.bashrc

```

Apply changes with the command below.

```
$ source ~/.bashrc

```

**This is it! Now you are done with SBC setup :)**  

Next Step : [OpenCR Setup](/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup "/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup")

Please refer to the Ubuntu Blog below for more useful information.

* [Improving Security with Ubuntu](https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu "https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu")
* [Improving User Experience of TurtleBot3 Waffle Pi](https://ubuntu.com/blog/building-a-better-turtlebot3 "https://ubuntu.com/blog/building-a-better-turtlebot3")
* [How to set up TurtleBot3 Waffle Pi in minutes with Snaps](https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps "https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps")

## [SBC Setup](#sbc-setup "#sbc-setup")

**WARNING**

* TurtleBot3 on Windows is running on a single PC assembled on TurtleBot3 instead of Raspberry Pi.
* If you have replaced Raspberry Pi with UP2 or Intel NUC and followed PC setup section on it, you can bypass this section.

 Previous Page
Next Page 
