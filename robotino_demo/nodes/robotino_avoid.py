#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import Twist
from robotino_demo.Controller import Controller
from robotino_demo.CollisionAvoidance import CollisionAvoidance


class Avoid():

	def __init__(self):

		# Init node
		rospy.init_node('robotino_avoid', anonymous=False)
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

		# Loop
		while not rospy.is_shutdown():

			# Get points in roi from laser data
			[points_in_left_roi, points_in_right_roi] = self.ca.compute_cartesian_from_laser() 

			# Compute control commands
			if(points_in_right_roi > self.ca.threshold and points_in_left_roi < self.ca.threshold):
				vel = 0.0
				omega = 40 * math.pi / 180
			elif(points_in_left_roi > self.ca.threshold and points_in_right_roi < self.ca.threshold):
				vel = 0.0
				omega = - 40 * math.pi / 180
			elif(points_in_left_roi > self.ca.threshold and points_in_right_roi > self.ca.threshold):
				vel = 0.0
				omega = - 40 * math.pi / 180
			else:
				vel = 0.3
				omega = 0.0

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
		Avoid()
		rospy.spin()
	except:
		rospy.loginfo("Shutdown program.")
