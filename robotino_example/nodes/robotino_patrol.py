#!/usr/bin/env python

import rospy
import tf
import math
import numpy as np
from math import sqrt, pow, pi, atan2
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class PatrolStop():

    def __init__(self):

        # Init node
        rospy.on_shutdown(self.shutdown)

        # Init publisher
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Set parameters for collision avoidance
        self.shift = rospy.get_param("/collision_avoidance/shift")
        self.range_limit = rospy.get_param("/collision_avoidance/range_limit")
        self.safetyzone_margin = rospy.get_param("/collision_avoidance/safetyzone_margin")
        self.robot_diameter = rospy.get_param("/collision_avoidance/robot_diameter")
        self.robot_laser_distance = rospy.get_param("/collision_avoidance/robot_laser_distance")
        self.threshold = rospy.get_param("/collision_avoidance/threshold")

        # Set parameters for path controller
        self.k_rho = rospy.get_param("/pure_pursuit/k_rho")
        self.k_alpha = rospy.get_param("/pure_pursuit/k_alpha")
        self.max_vel = rospy.get_param("/pure_pursuit/speed")
        self.pos_tol = rospy.get_param("/pure_pursuit/pos_tol")
        self.ang_tol = rospy.get_param("/pure_pursuit/pos_tol")

        # Init transfrom listener
        self.tf_listener = tf.TransformListener()

        # Cycle rate
        self.r = rospy.Rate(10)

        # Create patrol sequence
        points_hall = [{'x': 0.0, 'y': 0.7, 'rx':0, 'ry': 0, 'rz': 0, 'rw': 1}, 
                        {'x': 11.5, 'y': 1.7, 'rx': 0, 'ry': 1, 'rz': 0, 'rw': 0}]
        points_CP_square = [{'x': 4.0, 'y': -7.6, 'rx': 0, 'ry': 0, 'rz': 0, 'rw': 1}, 
                            {'x': 2.9, 'y': -7.6, 'rx': 0, 'ry': 1, 'rz': 0, 'rw': 0},
                            {'x': 2.9, 'y': -9.5, 'rx': 0, 'ry': 0, 'rz': 0, 'rw': 1},
                            {'x': 4.0, 'y': -9.5, 'rx': 0, 'ry': 1, 'rz': 0, 'rw': 0}]

        # Loop over positions
        while not rospy.is_shutdown():
            for point in points_CP_square:
                self.go_to_point(point)

    def go_to_point(self, point):

        # Get current position and theta
        (position, rotation) = self.get_odom()

        # Get required position (z = theta)
        goal_x = point['x']
        goal_y = point['y']

        # Compute distance to goal
        distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))

        # Drive towards goal
        while distance > self.pos_tol:

            # Get position
            (position, _) = self.get_odom()
            x_start = position.x
            y_start = position.y

            # Get goal angle
            alpha = self.normalizeAngle(atan2(goal_y - y_start, goal_x- x_start))

            # Get goal distance
            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            
            # Calculate contol output
            omega = self.Kalpha * alpha  # + Kbeta * beta
            if abs(alpha) > self.ang_tol:
                vu = 0.0
            else:
                vu = self.Krho * distance

            # Limit outputs
            vu = min(vu, self.max_vel)

            # Publish
            cmd = Twist()
            cmd.linear.x = vu
            cmd.angular.z = omega
            self.cmd_pub.publish(cmd)
            self.r.sleep()

        # Publish zero twist
        self.cmd_vel.publish(Twist())


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

    def get_odom(self):
        try:
            self.tf_listener.waitForTransform('odom', 'base_link', rospy.Time(), rospy.Duration(1.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between odom and base_link")
            rospy.signal_shutdown("tf Exception")
        try:
            (trans, rot) = self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
            rotation = euler_from_quaternion(rot)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), rotation[2])

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    @staticmethod
    def normalizeAngle(angle):
        angle1 = math.atan2(math.sin(angle), math.cos(angle))
        return angle1

if __name__ == '__main__':

    try:
        rospy.init_node('robotino_patrol_stop', anonymous=False)
        PatrolStop()
        rospy.spin()
    except:
        rospy.loginfo("Shutdown program.")