#!/usr/bin/env python3

import rospy
from robotino_core.agv.AGV_Main_ROS import AGV

if __name__== '__main__':

    # Start AGV
    rospy.init_node("robotino_15")
    AGV('172.21.15.90', 10015, 15, '172.21.212.136', 'Robotino15', 'Robotino15', 'kb')
    rospy.spin()