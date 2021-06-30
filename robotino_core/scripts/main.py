#!/usr/bin/env python3

import sys
#import rospy
#from robotino_core.agv.AGV_Main_ROS import AGV
from robotino_core.agv.AGV_Main import AGV

if __name__== '__main__':

    # Get id from command line
    if len(sys.argv) > 1:
        id = int(sys.argv[1])
        ta_agent = sys.argv[2] == 'True'
        ro_agent = sys.argv[3] == 'True'
        rm_agent = sys.argv[4] == 'True'
    else:
        id = 15
        ta_agent = False
        ro_agent = False
        rm_agent = False

    # Get location
    depots = [(10, 30), (10, 50), (10, 70)]
    loc = depots[id%3]

    # Start AGV ROS
    #rospy.init_node("robotino_16")
    #AGV('172.21.16.90', 10015, 16, '172.21.212.136', 'Robotino16', 'Robotino16', 'kb')
    #rospy.spin()

    # Start AGV
    AGV('localhost', 10000+id, id, 'localhost', 'matthias', 'matthias', 'kb', loc, ta_agent, ro_agent, rm_agent)