#!/usr/bin/env python

import rospy
import tf
import numpy as np
from math import sqrt, pow, pi, atan2
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion

msg = """
Control Robotino!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""

class GotoPoint():

    def __init__(self):

        # Init node
        rospy.init_node('robotino_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # Set parameters for path controller
        k_rho = rospy.get_param("/pure_pursuit/k_rho")
        k_alpha = rospy.get_param("/pure_pursuit/k_alpha")
        max_vel = rospy.get_param("/pure_pursuit/speed")
        pos_tol = rospy.get_param("/pure_pursuit/pos_tol")
        ang_tol = rospy.get_param("/pure_pursuit/pos_tol")

        # Init publisher
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Init transfrom listener
        self.tf_listener = tf.TransformListener()

        # Create destination point
        position = Point()
        move_cmd = Twist()

        # Cycle rate
        r = rospy.Rate(10)
        
        # Get current position and theta
        (position, rotation) = self.get_odom()

        # Get required position (z = theta)
        (goal_x, goal_y, goal_z) = self.getkey()
        if goal_z > 180 or goal_z < -180:
            print("you input wrong z range.")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)

        # Compute distance to goal
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance

        # Drive towards goal
        last_rotation = 0
        while distance > 0.05:

            # Get position
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            path_angle = atan2(goal_y - y_start, goal_x- x_start)

            # Normalize angle
            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation
            
            # Compute angular velocity
            move_cmd.angular.z = k_alpha * path_angle - rotation
            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)
            last_rotation = rotation

            # Compute linear velocity
            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            move_cmd.linear.x = min(k_rho * distance, max_vel)

            # Publish twist
            self.cmd_vel.publish(move_cmd)
            r.sleep()

        # Get position
        (position, rotation) = self.get_odom()

        # Turn to goal
        while abs(rotation - goal_z) > 0.05:

            # Get position
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5

            # Publish twist
            self.cmd_vel.publish(move_cmd)
            r.sleep()

        # Publish zero twist
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())

    def getkey(self):
        x, y, z = input("| x | y | z |\n").split()
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z

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


if __name__ == '__main__':

    try:
        while not rospy.is_shutdown():
            print(msg)
            GotoPoint()

    except:
        rospy.loginfo("Shutdown program.")

