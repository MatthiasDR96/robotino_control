#!/bin/bash
gnome-terminal -x bash -c "source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch robotino_navigation robotino_gmapping.launch"
