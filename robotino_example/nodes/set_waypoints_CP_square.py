#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool

if __name__ == '__main__':

	# Initialize node
	rospy.init_node("send_goal")

	# Define subscriber and publisher
	cmd_pub = rospy.Publisher('/set_goal', Pose, queue_size=10)

	pose1 = Pose()
	pose1.position.x = 4.0
	pose1.position.y = -7.6
	pose1.orientation.w = 1

	pose2 = Pose()
	pose2.position.x = 2.9
	pose2.position.y = -7.6
	pose2.orientation.y = 1

	pose3 = Pose()
	pose3.position.x = 2.9
	pose3.position.y = -9.5
	pose3.orientation.w = 1

	pose4 = Pose()
	pose4.position.x = 4.0
	pose4.position.y = -9.5
	pose4.orientation.y = 1

	# Wait for publishers to come up
	r = rospy.Rate(10)
	while cmd_pub.get_num_connections() == 0:
		r.sleep()

	# Wait for pose to be initialized in move_base
	initial_position = rospy.wait_for_message("initialpose", PoseWithCovarianceStamped)

	# Execute infinite loop
	while not rospy.is_shutdown():

		rospy.sleep(1)
		cmd_pub.publish(pose1)
		rospy.loginfo("Goal 1 sent")

		rospy.loginfo("Waiting to send new goal...")
		goal = rospy.wait_for_message('/goal_reached', Bool)

		rospy.sleep(1)
		cmd_pub.publish(pose2)
		rospy.loginfo("Goal 2 sent")

		rospy.loginfo("Waiting to send new goal...")
		rospy.wait_for_message('/goal_reached', Bool)

		rospy.sleep(1)
		cmd_pub.publish(pose3)
		rospy.loginfo("Goal 3 sent")

		rospy.loginfo("Waiting to send new goal...")
		rospy.wait_for_message('/goal_reached', Bool)

		rospy.sleep(1)
		cmd_pub.publish(pose4)
		rospy.loginfo("Goal 4 sent")

		rospy.loginfo("Waiting to send new goal...")
		rospy.wait_for_message('/goal_reached', Bool)

	rospy.spin()
