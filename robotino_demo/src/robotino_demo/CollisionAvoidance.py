import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

class CollisionAvoidance:

	def __init__(self):

		# Set parameters for collision avoidance
		self.shift = rospy.get_param("/collision_avoidance/shift")
		self.range_limit = rospy.get_param("/collision_avoidance/range_limit")
		self.safetyzone_margin = rospy.get_param("/collision_avoidance/safetyzone_margin")
		self.robot_diameter = rospy.get_param("/collision_avoidance/robot_diameter")
		self.robot_laser_distance = rospy.get_param("/collision_avoidance/robot_laser_distance")
		self.threshold = rospy.get_param("/collision_avoidance/threshold")

	def compute_cartesian_from_laser(self):

		# Get laser data
		scan = rospy.wait_for_message("/scan", LaserScan)
		roi_boundry_left = scan.angle_max + self.shift
		roi_boundry_right = scan.angle_min + self.shift
		
		# Convert from polar to cartesian
		amount_of_measurements = np.shape(scan.ranges)[0]
		theta = np.linspace(roi_boundry_right, roi_boundry_left, amount_of_measurements)
		ranges_roi = np.array(scan.ranges[0:np.shape(scan.ranges)[0]])
		ranges_roi[ranges_roi < (self.robot_diameter / 2)] = 10
		ranges_matrix = np.array([theta , ranges_roi])
		x = np.multiply(ranges_matrix[1][:], np.cos(ranges_matrix[0][:]))
		y = np.multiply(ranges_matrix[1][:], np.sin(ranges_matrix[0][:])) + self.robot_laser_distance
		ranges_cartesian = np.array([x , y])
		
		# Set limits of roi
		cartesian_left_limit = -(self.robot_diameter / 2) - self.safetyzone_margin
		cartesian_right_limit = (self.robot_diameter / 2) + self.safetyzone_margin

		# Points in roi
		ranges_left_roi = np.transpose(np.array([ranges_cartesian[:,i] for i in range(ranges_cartesian.shape[1]) if (ranges_cartesian[0,i] >= cartesian_left_limit) and (ranges_cartesian[0][i] <= 0)]))
		ranges_right_roi = np.transpose(np.array([ranges_cartesian[:,i] for i in range(ranges_cartesian.shape[1]) if (ranges_cartesian[0,i] <= cartesian_right_limit) and (ranges_cartesian[0][i] >= 0)]))

		# Points in roi and safety zone
		avoid_ranges_left = np.transpose(np.array([ranges_left_roi[:,i] for i in range(ranges_left_roi.shape[1]) if (ranges_left_roi[1][i] < self.range_limit)]))
		avoid_ranges_right = np.transpose(np.array([ranges_right_roi[:,i] for i in range(ranges_right_roi.shape[1]) if (ranges_right_roi[1][i] < self.range_limit)]))

		# Amount of points in left and right roi
		points_in_left_roi = len(np.transpose(avoid_ranges_left))
		points_in_right_roi = len(np.transpose(avoid_ranges_right))
		
		return [points_in_left_roi, points_in_right_roi]

