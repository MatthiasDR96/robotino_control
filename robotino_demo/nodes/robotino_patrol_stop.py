#!/usr/bin/env python
import rospy
import tf
import os
import yaml
from geometry_msgs.msg import Twist
from robotino_demo.Controller import Controller
from robotino_demo.CollisionAvoidance import CollisionAvoidance

class PatrolStop():

	def __init__(self):

		# Init node
		rospy.init_node('robotino_patrol_stop', anonymous=False)
		rospy.on_shutdown(self.shutdown)

		# Init publisher
		self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

		# Init transfrom listener
		self.tf_listener = tf.TransformListener()

		# Create traject controller
		self.controller = Controller()

		# Create collision avoidance 
		self.ca = CollisionAvoidance()

		# Cycle rate
		self.r = rospy.Rate(10)

		# Get point sequence
		this_dir = os.path.dirname(os.path.dirname(__file__))
		data_path = os.path.join(this_dir, "locations", rospy.get_param("/locations"))
		with open(data_path, 'r') as file:
			locations = yaml.load(file, Loader=yaml.FullLoader)

		# Loop over locations
		while not rospy.is_shutdown():
			for point in locations:
				self.go_to_point(point)

	def go_to_point(self, goal):

		# Turn towards goal
		error = 0.2
		while abs(error) > 0.1:

			# Get current pos
			cur_pos = self.controller.get_current_location()

			# Get goal pos
			goal_pos = self.controller.get_goal_location(goal)

			# Compute control commands
			vel, omega, error = self.controller.turn_towards_goal(cur_pos, goal_pos)

			# Publish command
			cmd = Twist()
			cmd.linear.x = vel
			cmd.angular.z = omega
			self.cmd_pub.publish(cmd)

		# Move towards goal
		error = 0.2
		while abs(error) > 0.1:

			# Get points in roi from laser data
			[points_in_left_roi, points_in_right_roi] = self.compute_cartesian_from_laser() 

			# Get current pos
			cur_pos = self.controller.get_current_location()

			# Get goal pos
			goal_pos = self.controller.get_goal_location(goal)

			# Collision avoidance
			if (points_in_right_roi + points_in_left_roi) > self.threshold:
				vel = 0.0
				omega = 0.0
			else:
				vel, omega, error = self.move_towards_goal(cur_pos, goal_pos)

			# Publish command
			cmd = Twist()
			cmd.linear.x = vel
			cmd.angular.z = omega
			self.cmd_pub.publish(cmd)

		# Turn towards goal
		error = 0.2
		while abs(error) > 0.1:

			# Compute control commands
			vel, omega, error = self.controller.turn_towards_angle(cur_pos, goal_pos)

			# Publish command
			cmd = Twist()
			cmd.linear.x = vel
			cmd.angular.z = omega
			self.cmd_pub.publish(cmd)
	
	def shutdown(self):
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)

if __name__ == '__main__':

	try:
		PatrolStop()
		rospy.spin()
	except:
		rospy.loginfo("Shutdown program.")