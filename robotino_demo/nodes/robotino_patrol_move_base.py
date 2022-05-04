#!/usr/bin/env python
import rospy
import os
import yaml
from geometry_msgs.msg import Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class Patrol():

	def __init__(self):

		# Init node
		rospy.init_node('robotino_patrol', anonymous=False)
		rospy.on_shutdown(self.shutdown)

		# Init publisher
		self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

		# Cycle rate
		self.r = rospy.Rate(10)

		# Get point sequence
		this_dir = os.path.dirname(os.path.dirname(__file__))
		data_path = os.path.join(this_dir, "locations", rospy.get_param("/robotino_patrol/locations"))
		with open(data_path, 'r') as file:
			locations = yaml.load(file, Loader=yaml.FullLoader)

		# Loop over locations
		while not rospy.is_shutdown():
			for point in locations:
				print(f'Move to point {locations[point]}')
				self.go_to_point(locations[point])

	def go_to_point(self, goal):

		# Create actionlib client
		client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		client.wait_for_server()

		# Define goal
		goal_ = MoveBaseGoal()
		goal_.target_pose.header.frame_id = "map"
		goal_.target_pose.header.stamp = rospy.Time.now()
		goal_.target_pose.pose.position.x = goal[0]
		goal_.target_pose.pose.position.y = goal[1]
		q = quaternion_from_euler(0.0, 0.0, goal[2])
		goal_.target_pose.pose.orientation.x = q[0]
		goal_.target_pose.pose.orientation.y = q[1]
		goal_.target_pose.pose.orientation.z = q[2]
		goal_.target_pose.pose.orientation.w = q[3]

		# Send goal
		client.send_goal(goal_)
		wait = client.wait_for_result()
		if not wait:
			rospy.logerr("Action server not available!")
			rospy.signal_shutdown("Action server not available!")
		else:
			return client.get_result()

	def shutdown(self):
		self.cmd_pub.publish(Twist())
		rospy.sleep(1)

if __name__ == '__main__':
		
	Patrol()
	rospy.spin()