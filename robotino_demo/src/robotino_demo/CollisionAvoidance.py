import rospy
import numpy as np
import matplotlib as mpl
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

class CollisionAvoidance:

	def __init__(self):

		# Set params
		self.shift = rospy.get_param("/robotino_patrol/collision_avoidance/shift")
		self.range_limit = rospy.get_param("/robotino_patrol/collision_avoidance/range_limit")
		self.safetyzone_margin = rospy.get_param("/robotino_patrol/collision_avoidance/safetyzone_margin")
		self.robot_diameter = rospy.get_param("/robotino_patrol/collision_avoidance/robot_diameter")
		self.robot_laser_distance = rospy.get_param("/robotino_patrol/collision_avoidance/robot_laser_distance")
		self.threshold = rospy.get_param("/robotino_patrol/collision_avoidance/threshold")

		# Canvas
		mpl.rcParams['toolbar'] = 'None'	
		self.fig, self.ax = plt.subplots()
		self.fig.canvas.toolbar_visible = False
		self.fig.canvas.header_visible = False
		self.fig.canvas.footer_visible = False

	def compute_cartesian_from_laser(self):

		# Wait for laser data
		scan = rospy.wait_for_message("/scan", LaserScan)
		roi_boundry_left = scan.angle_max + self.shift
		roi_boundry_right = scan.angle_min + self.shift
		
		# Get raw laser data
		number_of_measurements = np.shape(scan.ranges)[0]
		theta = np.linspace(roi_boundry_right, roi_boundry_left, number_of_measurements)
		ranges_roi = np.array(scan.ranges[0:np.shape(scan.ranges)[0]])

		# Remove point that lie inside the robot diameter
		indices_to_remove = [i for i in range(len(ranges_roi)) if ranges_roi[i] < (self.robot_diameter / 2)]
		theta = np.delete(theta, indices_to_remove)
		ranges_roi = np.delete(ranges_roi, indices_to_remove)

		# Convert from polar to cartesian
		ranges_matrix = np.array([theta , ranges_roi])
		x = np.multiply(ranges_matrix[1][:], np.cos(ranges_matrix[0][:]))
		y = np.multiply(ranges_matrix[1][:], np.sin(ranges_matrix[0][:])) + self.robot_laser_distance
		ranges_cartesian = np.array([x , y])
		
		# Set limits of roi
		cartesian_left_limit = -(self.robot_diameter / 2) - self.safetyzone_margin
		cartesian_right_limit = (self.robot_diameter / 2) + self.safetyzone_margin

		# Plot avoidance
		plt.ion()
		self.ax.add_patch(plt.Circle((0, 0), self.robot_diameter/2, color='k'))
		self.ax.add_patch(plt.Rectangle((cartesian_left_limit, 0), (self.robot_diameter + 2*self.safetyzone_margin), self.range_limit, edgecolor = 'red', facecolor = 'blue', fill=None,lw=2))
		self.ax.plot(ranges_cartesian[0][:], ranges_cartesian[1][:], 'r.')
		self.ax.set_xlim([-1,1])
		self.ax.set_ylim([0, 2])
		self.ax.axes.xaxis.set_visible(False)
		self.ax.axes.yaxis.set_visible(False)
		plt.draw()
		plt.pause(0.001)
		plt.cla()

		# Points in roi
		ranges_left_roi = np.transpose(np.array([ranges_cartesian[:,i] for i in range(ranges_cartesian.shape[1]) if (ranges_cartesian[0,i] >= cartesian_left_limit) and (ranges_cartesian[0][i] <= 0)]))
		ranges_right_roi = np.transpose(np.array([ranges_cartesian[:,i] for i in range(ranges_cartesian.shape[1]) if (ranges_cartesian[0,i] <= cartesian_right_limit) and (ranges_cartesian[0][i] >= 0)]))

		# Points in roi and safety zone
		avoid_ranges_left = np.transpose(np.array([ranges_left_roi[:,i] for i in range(ranges_left_roi.shape[1]) if (ranges_left_roi[1][i] < self.range_limit)]))
		avoid_ranges_right = np.transpose(np.array([ranges_right_roi[:,i] for i in range(ranges_right_roi.shape[1]) if (ranges_right_roi[1][i] < self.range_limit)]))

		# Number of points in left and right roi
		points_in_left_roi = len(np.transpose(avoid_ranges_left))
		points_in_right_roi = len(np.transpose(avoid_ranges_right))
		
		return [points_in_left_roi, points_in_right_roi]

