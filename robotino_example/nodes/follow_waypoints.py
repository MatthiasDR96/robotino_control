#!/usr/bin/env python

import numpy as np
import rospy
import math
import tf
import PyKDL
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool

# Init global variables
goal_reached = False


def calculateControlOutput(goal):

	# Get current position wrt map frame
	tf = TransformStamped()
	listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(20.0))
	(trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
	tf.transform.translation = trans
	tf.transform.rotation = rot

	# Goal in base link frame
	goal_in_base_link_frame = transformToBaseLink(goal, tf.transform)
	distance_to_goal = math.sqrt(math.pow(goal_in_base_link_frame.p[0],2) + math.pow(goal_in_base_link_frame.p[1],2))

	global goal_reached

	if distance_to_goal < pos_tol:
		rospy.loginfo("Goal is reached")
		goal_reached_cmd = Bool()
		goal_reached_cmd.data = True
		goal_reached_pub.publish(goal_reached_cmd)
		goal_reached = True
		return [0, 0]

	else:

		# Get goal position
		xg = goal_in_base_link_frame.p[0]
		yg = goal_in_base_link_frame.p[1]
		thetag = goal_in_base_link_frame.M.GetEulerZYX()[0]
		#plt.arrow(0, 0, 0.1, 0)
		#plt.arrow(xg, yg, math.cos(thetag), math.sin(thetag))
		#plt.show()

		# Compute control quantities
		rho = math.sqrt(math.pow(xg,2) + math.pow(yg,2))
		alpha = math.atan2(yg, xg)
		alpha = normalizeAngle(alpha)

		#beta = - alpha + thetag
		#beta = normalizeAngle(beta)

		if alpha > 0.1 or alpha < -0.1:
			print("Turn to goal")
			vu = 0.0
			omega = Kalpha * alpha #+ Kbeta * beta
		else:
			print("Move towards goal")
			vu = Krho * rho
			omega = Kalpha * alpha #+ Kbeta * beta

			# Can also drive backwards
			#if abs(alpha) < math.pi / 2:
				#alpha = alpha - math.pi
				#alpha = normalizeAngle(alpha)
				#beta = thetag - alpha - math.pi
				#vu = - Krho * rho
				#omega = Kalpha * alpha #+ Kbeta * beta
	
			# Keep velocity constant
	
		# Keep velocity constant
		tol = 1e-6
		absVel = abs(vu)
		if absVel > tol:
			vu = vu / absVel * speed
			omega = omega / absVel * speed

		return [vu, omega]


def normalizeAngle(angle):
	
	angle1 = math.atan2(math.sin(angle), math.cos(angle))
	return angle1


def transformToBaseLink(pose, tf):

	# Pose in global frame
	f_map_pose = PyKDL.Frame(PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w), PyKDL.Vector(pose.position.x,pose.position.y,pose.position.z))
	
	# Robot in global map frame
	f_map_tf = PyKDL.Frame(PyKDL.Rotation.Quaternion(tf.rotation[0],tf.rotation[1],tf.rotation[2],tf.rotation[3]), PyKDL.Vector(tf.translation[0], tf.translation[1], tf.translation[2]))
	return PyKDL.Frame.Inverse(f_map_tf) * f_map_pose


if __name__ == "__main__":

	# Initialize node
	rospy.init_node("follow_waypoints_node")

	# Set parameters for path controller
	Krho = rospy.get_param("/pure_pursuit/k_rho")
	Kalpha = rospy.get_param("/pure_pursuit/k_alpha")
	Kbeta = rospy.get_param("/pure_pursuit/k_beta")
	speed = rospy.get_param("/pure_pursuit/speed")
	pos_tol = rospy.get_param("/pure_pursuit/pos_tol")

	# Define vel publisher
	cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	goal_reached_pub = rospy.Publisher('/goal_reached', Bool, queue_size=10)

	# Define listener
	listener = tf.TransformListener()

	# Init
	goal_reached = False

	while not rospy.is_shutdown():
	
		rospy.loginfo("Waiting for goal...")
		goal = rospy.wait_for_message('/set_goal', Pose)

		while not goal:
			cmd = Twist()
			cmd_pub.publish(cmd)

		rospy.loginfo("Goal received")
		while not goal_reached:
			[vel, omega] = calculateControlOutput(goal)
			cmd = Twist()
			cmd.linear.x = vel
			cmd.angular.z = omega
			cmd_pub.publish(cmd)

		goal_reached= False

	rospy.spin()
