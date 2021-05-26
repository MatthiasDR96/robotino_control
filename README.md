# Robotino
A package containing all code for the operation of the Robotino AGVs in the Ultimate Factory in Bruges.

## Installation

In order to use the robot you first have to install Ubuntu Focal (20.04) as well as ROS Noetic, which currently is the only supported ROS version of this repository. After installing Ubuntu on a computer please follow the instructions below for installing ROS and all other necessary software.

```bash
#!/bin/bash

# Define config variables
ROS_DISTRO=noetic 
ROS_OS=ubuntu:focal
ROS_CI_DESKTOP=`lsb_release -cs`  # e.g. [trusty|xenial|...]

# Install ROS and all necessary dependencies
sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu $ROS_CI_DESKTOP main\" > /etc/apt/sources.list.d/ros-latest.list"
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
wget -qO - http://packages.openrobotino.org/keyFile | sudo apt-key add -
sudo sh -c "echo \"deb http://packages2.openrobotino.org focal main\" > /etc/apt/sources.list.d/openrobotino.list"
sudo apt update -qq
sudo apt install -y dpkg rec-rpc robotino-dev robotino-api2 
sudo apt install -y git python3-pip python3-catkin-pkg python3-rosdep ros-$ROS_DISTRO-ros-base

# Install catkin tools using either one of these methods (only one works ususally, try it out in the worst case)
sudo apt install -y python3-catkin-tools

# Source the just installed ROS environment and initialize rosdep
source /opt/ros/$ROS_DISTRO/setup.bash
sudo rosdep init
rosdep update

# Create a catkin workspace with the package under integration.
mkdir -p ~/robotino_ws/src
cd ~/robotino_ws/src
catkin_init_workspace

# Create the devel/setup.bash (run catkin_make with an empty workspace) and
# source it to set the path variables.
cd ~/robotino_ws
catkin_make
source devel/setup.bash

# Download the code for rto_core and rto_simulation
cd ~/robotino_ws/src
git clone https://github.com/dietriro/rto_core.git
git clone https://github.com/dietriro/rto_simulation.git

# Download the code for this repository
cd ~/robotino_ws/src
git clone https://github.com/MatthiasDR96/robotino.git

# Download the code for the SICK laser scanner
cd ~/robotino_ws/src
git clone https://github.com/bohlender/sicks300.git
sudo chmod +x /dev/ttyUSB0
reboot

# Download the code for the webcam
cd ~/robotino_ws/src
git clone https://github.com/ros-drivers/usb_cam.git
sudo apt-get install ros-noetic-image-view

# Install dependencies using rosdep
cd ~/robotino_ws
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --os=$ROS_OS

# Build the workspace
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/robotino_ws
catkin_make
```

After finishing the setup and sucessfully building the workspace, you need to either source the just built workspace everytime you use it or you just add a line to your `.bashrc` so that it is source everytime a new console is opened. I recommend appending the following lines to the end of your `~/.bashrc` file:

```bash
# ROS Setup
source ~/robotino_ws/devel/setup.bash
```