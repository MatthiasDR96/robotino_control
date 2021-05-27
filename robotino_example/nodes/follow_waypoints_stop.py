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
goal = Pose()

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

		# Compute control quantities
		rho = math.sqrt(math.pow(xg,2) + math.pow(yg,2))
		alpha = math.atan2(yg, xg)
		alpha = normalizeAngle(alpha)

		if alpha > 0.1 or alpha < -0.1:
			print("Turn to goal")
			vu = 0.0
			omega = Kalpha * alpha 
		else:
			print("Move towards goal")
			vu = Krho * rho
			omega = Kalpha * alpha 

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


def collisionAvoidanceController():

	# Get laser data
	#print("Waiting for scan message...")
	scan = rospy.wait_for_message("/scan", LaserScan)
	roi_boundry_left = scan.angle_max + shift
	roi_boundry_right = scan.angle_min + shift
	
	# Convert from polar to cartesian
	amount_of_measurements = 481
	theta = np.linspace(roi_boundry_right, roi_boundry_left, amount_of_measurements)
	ranges_roi = np.array(scan.ranges[0:481])
	ranges_roi[ranges_roi < (robot_diameter / 2)] = 10
	ranges_matrix = np.array([theta , ranges_roi])
	x = np.multiply(ranges_matrix[1][:], np.cos(ranges_matrix[0][:]))
	y = np.multiply(ranges_matrix[1][:], np.sin(ranges_matrix[0][:])) + robot_laser_distance
	ranges_cartesian = np.array([x , y])
	
	# Set limits of roi
	cartesian_left_limit = -(robot_diameter / 2) - safetyzone_margin
	cartesian_right_limit = (robot_diameter / 2) + safetyzone_margin

	# Points in roi
	ranges_left_roi = np.transpose(np.array([ranges_cartesian[:,i] for i in range(ranges_cartesian.shape[1]) if (ranges_cartesian[0,i] >= cartesian_left_limit) & (ranges_cartesian[0][i] < 0)]))
	ranges_right_roi = np.transpose(np.array([ranges_cartesian[:,i] for i in range(ranges_cartesian.shape[1]) if (ranges_cartesian[0,i] <= cartesian_right_limit) & (ranges_cartesian[0][i] >= 0)]))

	# Points in roi and safety zone
	avoid_ranges_left = np.transpose(np.array([ranges_left_roi[:,i] for i in range(ranges_left_roi.shape[1]) if (ranges_left_roi[1][i] < range_limit)]))
	avoid_ranges_right = np.transpose(np.array([ranges_right_roi[:,i] for i in range(ranges_right_roi.shape[1]) if (ranges_right_roi[1][i] < range_limit)]))

	# Amount of points in left and right roi
	points_in_left_roi = len(np.transpose(avoid_ranges_left))
	points_in_right_roi =len(np.transpose(avoid_ranges_right))

	# Collision avoidance
	if (points_in_right_roi + points_in_left_roi) > threshold:
		print("Stopped")
		cmd = Twist()
		cmd_pub.publish(cmd)
	else:
		print("Continue path")
		[vel, omega] = calculateControlOutput(goal)
		cmd = Twist()
		cmd.linear.x = vel
		cmd.angular.z = omega
		cmd_pub.publish(cmd)


def main():

	global goal_reached
	global goal

	while not rospy.is_shutdown():

		rospy.loginfo("Waiting for goal...")
		goal = rospy.wait_for_message('/set_goal', Pose)
		rospy.loginfo("Goal received")
		while not goal_reached:
			collisionAvoidanceController()
		goal_reached= False


if __name__ == "__main__":

	# Initialize node
	rospy.init_node("follow_waypoints_CA_node")

	# Set parameters for collision avoidance
	shift = rospy.get_param("/collision_avoidance/shift")
	range_limit = rospy.get_param("/collision_avoidance/range_limit")
	safetyzone_margin = rospy.get_param("/collision_avoidance/safetyzone_margin")
	robot_diameter = rospy.get_param("/collision_avoidance/robot_diameter")
	robot_laser_distance = rospy.get_param("/collision_avoidance/robot_laser_distance")
	threshold = rospy.get_param("/collision_avoidance/threshold")

	# Set parameters for path controller
	Krho = rospy.get_param("/pure_pursuit/k_rho")
	Kalpha = rospy.get_param("/pure_pursuit/k_alpha")
	Kbeta = rospy.get_param("/pure_pursuit/k_beta")
	speed = rospy.get_param("/pure_pursuit/speed")
	pos_tol = rospy.get_param("/pure_pursuit/pos_tol")

	# Define publishers
	cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	goal_reached_pub = rospy.Publisher('/goal_reached', Bool, queue_size=10)

	# Define transform listener
	listener = tf.TransformListener()

	main()

	rospy.spin()
