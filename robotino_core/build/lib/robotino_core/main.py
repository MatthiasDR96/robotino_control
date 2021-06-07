import rospy
from agv.AGV_Main_ROS import AGV

if __name__== '__main__':

    # Start AGV
    rospy.init_node('robotino15_node')
    AGV('localhost', 10001, 15, '172.21.212.136', 'Robotino15', 'Robotino15', 'kb')
    rospy.spin()
