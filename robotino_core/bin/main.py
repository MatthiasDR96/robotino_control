#import rospy
from robotino_core.agv.AGV_Main import AGV

if __name__== '__main__':

    # Start AGV
    #rospy.init_node('robotino_node')
    #AGV('localhost', 10001, 15, '172.21.212.136', 'Robotino15', 'Robotino15', 'kb')
    AGV('localhost', 10001, 15, 'localhost', 'matthias', 'matthias', 'kb', (10, 30))
    #rospy.spin()