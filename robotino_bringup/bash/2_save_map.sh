#!/bin/bash
gnome-terminal -x bash -c "source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; rosrun map_server map_saver -f ~/catkin_ws/src/robotino_control/robotino_navigation/maps/new_map
"
