import rospy
import numpy as np
#import matplotlib as mpl
from sensor_msgs.msg import LaserScan
#import matplotlib.pyplot as plt

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
		#mpl.rcParams['toolbar'] = 'None'	
		#self.fig, self.ax = plt.subplots()
		#self.fig.canvas.toolbar_visible = False
		#self.fig.canvas.header_visible = False
		#self.fig.canvas.footer_visible = False

	def compute_cartesian_from_laser(self):

		# Wait for laser data
		scan = rospy.wait_for_message("/scan", LaserScan)
		roi_boundry_left = scan.angle_max + self.shift
		roi_boundry_right = scan.angle_min + self.shift
		
		# Get raw laser data
		number_of_measurements = np.shape(scan.ranges)[0]
		theta = np.linspace(roi_boundry_right, roi_boundry_left, number_of_measurements)
		ranges_roi = np.array(scan.ranges[0:np.shape(scan.ranges)[0]])

		# Convert from polar to cartesian
		x = np.multiply(ranges_roi, np.cos(theta))
		y = np.multiply(ranges_roi, np.sin(theta)) + self.robot_laser_distance

		# Remove points that lie inside the robot diameter
		indices_to_remove = [i for i in range(len(ranges_roi)) if x[i]**2 + y[i]**2 < (self.robot_diameter / 2)**2]
		x = np.delete(x, indices_to_remove)
		y = np.delete(y, indices_to_remove)
		ranges_cartesian = np.transpose([x, y])
		
		# Set limits of roi
		cartesian_left_limit = -(self.robot_diameter / 2) - self.safetyzone_margin
		cartesian_right_limit = (self.robot_diameter / 2) + self.safetyzone_margin

		# Points within safety zone
		ranges_cartesian_roi = ranges_cartesian[np.where(ranges_cartesian[:,1] <= self.range_limit)]

		# Points in left and right roi
		ranges_cartesian_roi_left = ranges_cartesian_roi[np.where(ranges_cartesian_roi[:,0] >= cartesian_left_limit)]
		ranges_cartesian_roi_left = ranges_cartesian_roi_left[np.where(ranges_cartesian_roi_left[:,0] <= 0)] 
		ranges_cartesian_roi_right = ranges_cartesian_roi[np.where(ranges_cartesian_roi[:,0] <= cartesian_right_limit)]
		ranges_cartesian_roi_right = ranges_cartesian_roi_right[np.where(ranges_cartesian_roi_right[:,0] >= 0)]

		# Number of points in left and right roi
		points_in_left_roi = len(ranges_cartesian_roi_left)
		points_in_right_roi = len(ranges_cartesian_roi_right)

		# Plot avoidance
		#plt.ion()
		#self.ax.add_patch(plt.Circle((0, 0), self.robot_diameter/2, color='k'))
		#self.ax.add_patch(plt.Rectangle((cartesian_left_limit, 0), (self.robot_diameter + 2*self.safetyzone_margin), self.range_limit, edgecolor = 'red', facecolor = 'blue', fill=None,lw=2))
		#self.ax.plot(x, y, 'r.')
		#self.ax.set_xlim([-1,1])
		#self.ax.set_ylim([0, 2])
		#self.ax.axes.xaxis.set_visible(False)
		#self.ax.axes.yaxis.set_visible(False)
		#plt.draw()
		#plt.pause(0.001)
		#plt.cla()
		
		return [points_in_left_roi, points_in_right_roi]

