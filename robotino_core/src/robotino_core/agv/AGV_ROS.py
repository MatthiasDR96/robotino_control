import time
import tf
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from robotino_core.agv.AGV_Main import AGV_Main


class AGV_ROS(AGV_Main):

	"""
		A class containing the intelligence of the agv agent
	"""

	def __init__(self, params_file):

		# AGV init
		super().__init__(params_file)

		# Overwrite action layer
		self.action.move_to_pos = self.move_to_pos
		self.action.cancel_goal = self.cancel_goal
		self.action.pick = self.pick
		self.action.place = self.place

		# Action client
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		#self.server = actionlib.SimpleActionServer('move_base', MoveBaseAction, self.active_cb, auto_start=False)
		#self.server.start()
		
		# Init transfrom listener
		self.tf_listener = tf.TransformListener()

		# Init subscriber
		self.status_sub = rospy.Subscriber("/odom", Odometry, self.amcl_callback)

		# Shutdown
		rospy.on_shutdown(self.shutdown)

	def shutdown(self):

		# Closing threads
		print("\nShutdown robot")

		# Cancel goal
		self.client.cancel_all_goals()

	def amcl_callback(self, msg):

		try:
			self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(), rospy.Duration(1.0))
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			rospy.loginfo("Cannot find transform between odom and base_link")
			rospy.signal_shutdown("tf Exception")
		try:
			(trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
			rotation = euler_from_quaternion(rot)
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			rospy.loginfo("TF Exception")

		# Get position
		self.x_loc = trans[0] # msg.pose.pose.position.x
		self.y_loc = trans[1] # msg.pose.pose.position.y

		# Get orientation
		# orientation_q = msg.pose.pose.orientation
		# orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		# (_, _, yaw) = euler_from_quaternion(rot)
		self.theta = rotation[2] # yaw

		# Search closest node
		self.node = self.search_closest_node((self.x_loc, self.y_loc))

		# Battery status
		pass

	def move_to_pos(self, node_position):
		
		# Get theta
		print("Move to pos")
		rotation = quaternion_from_euler(0, 0, node_position[2])

		# Connect to server
		wait = self.client.wait_for_server(rospy.Duration(5.0))
		if not wait:
			rospy.logerr("Action server not available!")
			rospy.signal_shutdown("Action server not available!")
			return

		# Create goal
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = node_position[0]
		goal.target_pose.pose.position.y = node_position[1]
		goal.target_pose.pose.orientation.x = rotation[0]
		goal.target_pose.pose.orientation.y = rotation[1]
		goal.target_pose.pose.orientation.z = rotation[2]
		goal.target_pose.pose.orientation.w = rotation[3]

		# Send goal
		self.client.send_goal(goal)
		wait = self.client.wait_for_result()
		if not wait:
			rospy.logerr("Action server not available!")
			rospy.signal_shutdown("Action server not available!")
		else:
			return self.client.get_result()

	def cancel_goal(self):
		self.client.cancel_goal()

	def active_cb(self):
		rospy.loginfo("Goal pose is now being processed by the Action Server...")

	def feedback_cb(self, feedback):
		rospy.loginfo("Feedback for goal pose received: " + str(feedback))

	def done_cb(self, status, result):
		if status == 2:
			rospy.loginfo("Goal pose received a cancel request after it started executing, completed execution!")
		if status == 3:
			rospy.loginfo("Goal pose reached") 
			rospy.signal_shutdown("Final goal pose reached!")
			return
		if status == 4:
			rospy.loginfo("Goal pose was aborted by the Action Server")
			rospy.signal_shutdown("Goal pose aborted, shutting down!")
			return
		if status == 5:
			rospy.loginfo("Goal pose has been rejected by the Action Server")
			rospy.signal_shutdown("Goal pose rejected, shutting down!")
			return
		if status == 8:
			rospy.loginfo("Goal pose received a cancel request before it started executing, successfully cancelled!")

	def pick(self):
		print("AGV " + str(self.id) + ":        Pick")

	def place(self):
		print("AGV " + str(self.id) + ":        Place")
