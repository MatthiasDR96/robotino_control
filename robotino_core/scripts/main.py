#!/usr/bin/env python3

import rospy
from robotino_core.agv.AGV_Main_ROS import AGV

if __name__== '__main__':

    # Start AGV
    rospy.init_node("robotino_16")
    AGV('172.21.16.90', 10015, 16, '172.21.212.136', 'Robotino16', 'Robotino16', 'kb')
    rospy.spin()