# Boatlette - Autonomous Sailing Platform - Raspberry Pi setup
This is a guide to setup the Raspberry Pi 4 with Ubuntu (Xubuntu desktop), ROS (Melodic) and Arduino Mega 2560, which is utilized in this [autonomous sailing platform project](https://github.com/H8ste/SailBoatROS) - A project by Medialogy Msc students from Aalborg University.

Link to the other repositories: [Arduino Repository](https://github.com/H8ste/SailBoatArduinoInterface) & [Mechanical Repository](https://github.com/H8ste/BoatletteSailBoat-Meca) & [Visualisation Repository](https://github.com/H8ste/Boatplatform-visualisation).

Known issues and usage of the platform are included at the end of the guide.

Before starting this guide, make sure you have:
* A Raspberry Pi 4
* A ethernet cable
* A monitor that supports HDMI input
* USB Keyboard
* USB mouse

## Step by step - Ubuntu Server on Raspberry Pi 4 with Arduino and ROS

### Update firmware with Raspbian

1. Download a [Raspbian image](https://www.raspberrypi.org/downloads/raspbian/). We used `Raspbian Buster with desktop and recommended software` (version: September 2019, release: 2019-09-26, kernel: 4.19), but any version should do just fine.

2. Flash the image on a SD card with a SD card flasher software. We used [Balena Etcher](https://www.balena.io/etcher/) (version 1.5.58).

3. Install Rasbian and update the firmware with the following commands:
```
sudo apt-get update && sudo apt-get dist-upgrade -y
```
```
sudo rpi-update
```

and afterwards
```
sudo rpi-eeprom-update -a
```

### Ubuntu installation
Now that the firmware is up-to-date, Ubuntu Server can now be setup.

1. Download the unofficial image from [here](https://jamesachambers.com/raspberry-pi-4-ubuntu-server-desktop-18-04-3-image-unofficial/)

2. Flash the image on a SD card with a SD card flasher software. We used [Balena Etcher](https://www.balena.io/etcher/) (version 1.5.58).

3. Login with username: `ubuntu` and password: `ubuntu`. You will be prompted to set a new password afterwards.

4. Check if date is correct with `date`. If it isn't, set the date manually with for now:  
```
sudo date –s ‘2019-12-14 11:25:50’ 
```

5. Run updater provided by [James A. Chambers](https://jamesachambers.com/raspberry-pi-4-ubuntu-server-desktop-18-04-3-image-unofficial/): 
```
wget https://raw.githubusercontent.com/TheRemote/Ubuntu-Server-raspi4-unofficial/master/Updater.sh 
```
```
chmod +x Updater.sh 
```
```
sudo ./Updater.sh 
```

6. Reboot and set date and time again if necessary. 

7. Install xubuntu desktop (we had version 18.04): 
```
sudo apt-get update && sudo apt-get dist-upgrade -y && sudo apt-get install xubuntu-desktop –y 
```
8. Reboot to start the desktop. Fix time/date permanently with the commands below. **NOTE:** This will not work if you have set the correct time manually.
```
sudo apt-get install htpdate 
```
```
sudo htpdate -a google.com 
```

## Installing and setting up ROS and Arduino

1. Install ROS Melodic by following [this guide](http://wiki.ros.org/melodic/Installation/Ubuntu) 

2. Setup a workspace for ROS according to [this documentation](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

3. Install python3-pip and python3-yaml:  
```
sudo apt install python3-pip python3-yaml 
```
4. Install the following ros-related python packages through pip3-install: 
```
pip3 install rospkg catkin_pkg 
```
5. Clone our github for the ROS-files (you can also use [Plymouth SailboatROS](https://github.com/Plymouth-Sailboat), as it gets updated often):  
```
cd ~/catkin_ws/src 
```
```
git clone https://github.com/H8ste/SailBoatROS.git 
```
6. Update dependencies: 
```
cd ~/catkin_ws/ 
```
```
rosdep update 
```
```
rosdep install --from-paths src --ignore-src --rosdistro melodic -y  
```
```
sudo apt install libnlopt-dev python-numpy python-pygame 
```
```
sudo apt install python-pip 
```
```
pip install -r src/SailBoatROS/requirements.txt  
```
```
pip3 install -r src/SailBoatROS/requirements.txt  
```
7. Configure our environment, so we can use the commands in ROS: 
```
cd ~/catkin_ws/ 
```
```
catkin_make 
```
```
source devel/setup.bash 
```
8. Download [Arduino IDE for Linux ARM 64 bits](https://www.arduino.cc/en/main/software). We used version 1.8.10.

9. Clone our github for the arduino files. You can also instead use the [Plymouth Arduino Interface](https://github.com/Plymouth-Sailboat/SailBoatArduinoInterface) which is the basis of our Sailboat: 
```
git clone https://github.com/H8ste/SailBoatArduinoInterface 
```

10. Copy `libraries`-folder from the sailboat-arduino-github above into your own Arduino installation directory. 

11. Acquire up-to-date library from ROS with the command below and replace the `ros_lib`-library in your `libraries`-folder at the Arduino installation directory:  
```
rosrun rosserial_arduino make_libraries.py 
```
12. Go to `arduino-1-8-10/libraries/Sailboat/` and add own config-file (copy `config-Boatlette.h` and change accordingly to own sailboat configuration using the naming convention `config-YourBoatName.h`). Add the following code to `config-Sailboat.h` afterwards to utilize the new config-file: 
```
#ifdef SAILBOAT_YOURBOATNAME 
#pragma message("SAILBOAT_ YOURBOATNAME chosen") 
#include <config-YourBoatName.h> 
#endif 
```

## Logging data

Our autonomous sailboat platform affords the ability to log data through use of a node-express back-end application. 


1. Install requests-module to be able to receive http-requests: 
```
pip install requests 
```
2. Clone rospy-message-converter into `catkin_ws/src`: 
```
cd ~/catkin_ws/src 
```
```
git clone https://github.com/uos/rospy_message_converter.git 
```

##  Know issues

* If you are dependant of a wifi connection while setting up the Raspberry Pi, use the following guide to fix the wifi adapter: https://www.linuxbabe.com/command-line/ubuntu-server-16-04-wifi-wpa-supplicant 

* Fix time and date by running the commands below, (**NOTE:** the date/time should not be correct set when doing this!): 
```
sudo apt-get install htpdate 
```
```
sudo htpdate -a google.com 
```
* When booting up the Raspberry Pi, it waits for the network and will continue to do so for 2 min. To disable this, run the following: 
```
systemctl disable systemd-networkd-wait-online.service 
```
```
systemctl mask systemd-networkd-wait-online.service 
```
##  Using the software

To run ROS, use the following commands in the ubuntu terminal, or by SSH:
```
roslaunch sailboat start.launch
```

Then run your chosen navigational algorithm, based the software architechture from the [Plymouth Sailboat](https://github.com/Plymouth-Sailboat), for instance:

Waypoint follower:
```
rosrun sailrobot waypoint_follower
```
Line-following:
```
rosrun sailrobot line_following_node
```
