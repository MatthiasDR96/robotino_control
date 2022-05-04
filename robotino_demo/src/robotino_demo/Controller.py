import tf
import rospy
import math
from tf.transformations import euler_from_quaternion

class Controller():

	def __init__(self):

		# Init transfrom listener
		self.tf_listener = tf.TransformListener()

		# Read params
		self.k_rho = rospy.get_param("/robotino_patrol/pure_pursuit/k_rho")
		self.k_alpha = rospy.get_param("/robotino_patrol/pure_pursuit/k_alpha")
		self.max_vel = rospy.get_param("/robotino_patrol/pure_pursuit/speed")
		self.pos_tol = rospy.get_param("/robotino_patrol/pure_pursuit/pos_tol")
		self.ang_tol = rospy.get_param("/robotino_patrol/pure_pursuit/pos_tol")

	def get_current_location(self):

		# Get current position wrt map frame
		self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(20.0))
		(trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))

		# Get 3D pose
		x = trans[0]
		y = trans[1]
		z = euler_from_quaternion(rot)[2]

		return [x, y, z]

	def get_goal_location(self, goal):

		# Get 3D pose
		x = goal[0]
		y = goal[1]
		z = goal[2]

		return [x, y, z]

	def turn_towards_goal(self, cur_pos, goal_pos):

		# Calculate difference
		delta_x = goal_pos[0] - cur_pos[0]
		delta_y = goal_pos[1] - cur_pos[1]

		# Compute angle towards goal
		alpha = math.atan2(delta_y, delta_x)

		# Normalize angle
		alpha = self.normalize_angle(alpha)

		# Compute error
		error = alpha - cur_pos[2]

		print(f'{alpha*180/math.pi} - {cur_pos[2]*180/math.pi} = {error*180/math.pi}')

		# Compute control signels
		vel = 0.0
		omega = self.k_alpha * error

		return vel, omega, error

	def turn_towards_angle(self, cur_pos, goal_pos):

		# Compute error
		error = goal_pos[2] - cur_pos[2]

		# Normalize angle
		error = self.normalize_angle(error)

		# Compute control signels
		vel = 0.0
		omega = self.k_alpha * error

		return vel, omega, error

	def move_towards_goal(self, cur_pos, goal_pos):

		# Calculate difference
		delta_x = goal_pos[0] - cur_pos[0]
		delta_y = goal_pos[1] - cur_pos[1]

		# Compute angle towards goal
		alpha = math.atan2(delta_y, delta_x)

		# Normalize angle
		alpha = self.normalize_angle(alpha)

		# Compute error angle
		alpha = alpha - cur_pos[2]

		# Compute error distance
		rho = math.sqrt(math.pow(delta_x,2) + math.pow(delta_y,2))
		
		# Compute control commands
		vel = self.k_rho * rho
		omega = self.k_alpha * alpha
		
		# Keep velocity constant
		tol = 1e-6
		absVel = abs(vel)
		if absVel > tol:
			vel = vel / absVel * self.max_vel
			omega = omega / absVel * self.max_vel

		# Limit commands
		vel = min(vel, self.max_vel)

		return vel, omega, rho

	def normalize_angle(self, angle):
		angle1 = math.atan2(math.sin(angle), math.cos(angle))
		return angle1
