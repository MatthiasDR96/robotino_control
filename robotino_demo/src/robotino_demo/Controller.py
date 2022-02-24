import rospy
import math
from tf.transformations import euler_from_quaternion

class Controller():

	def __init__(self):

		# Read params
		self.k_rho = rospy.get_param("/pure_pursuit/k_rho")
		self.k_alpha = rospy.get_param("/pure_pursuit/k_alpha")
		self.max_vel = rospy.get_param("/pure_pursuit/speed")
		self.pos_tol = rospy.get_param("/pure_pursuit/pos_tol")
		self.ang_tol = rospy.get_param("/pure_pursuit/pos_tol")

	def get_current_location(self):

		# Get current position wrt map frame
		self.listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(20.0))
		(trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))

		# Get 3D pose
		x = trans[0]
		y = trans[1]
		z = euler_from_quaternion(rot)[2]

		return [x, y, z]

	def get_goal_location(self, goal):

		# Get 3D pose
		x = goal[0]
		y = goal[1]
		z = euler_from_quaternion(goal[3:])[2]

		return [x, y, z]

	def turn_towards_goal(self, cur_pos, goal_pos):

		# Calculate difference
		delta_x = goal_pos.x - cur_pos.x
		delta_y = goal_pos.y - cur_pos.y

		# Compute angle towards goal
		alpha = math.atan2(delta_y, delta_x)

		# Normalize angle
		alpha = self.normalize_angle(alpha)

		# Compute error
		error = alpha - cur_pos[2]

		# Compute control signels
		v = 0.0
		omega = self.k_alpha * error

		# Limit commands
		vel = min(vel, self.max_vel)
		omega = min(omega, self.max_omega)

		return v, omega, error

	def turn_towards_angle(self, cur_pos, goal_pos):

		# Compute error
		error = goal_pos[2] - cur_pos[2]

		# Normalize angle
		error = self.normalize_angle(error)

		# Compute control signels
		v = 0.0
		omega = self.k_alpha * error

		# Limit commands
		vel = min(vel, self.max_vel)
		omega = min(omega, self.max_omega)

		return v, omega, error

	def move_towards_goal(self, cur_pos, goal_pos):

		# Calculate difference
		delta_x = goal_pos.x - cur_pos.x
		delta_y = goal_pos.y - cur_pos.y

		# Compute angle towards goal
		alpha = math.atan2(delta_y, delta_x)

		# Normalize angle
		alpha = self.normalize_angle(alpha)

		# Compute error angle
		alpha = alpha - cur_pos[2]

		# Compute error distance
		rho = math.sqrt(math.pow(delta_x,2) + math.pow(delta_y,2))
		
		# Compute control commands
		if alpha > 0.1 or alpha < -0.1:
			print("Turn to goal")
			vel = 0.0
			omega = self.k_alpha * alpha
		else:
			print("Move towards goal")
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
		omega = min(omega, self.max_omega)

		return vel, omega, rho

	def normalize_angle(angle):
	
		angle1 = math.atan2(math.sin(angle), math.cos(angle))
		return angle1
