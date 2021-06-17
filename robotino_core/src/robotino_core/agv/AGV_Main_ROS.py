from datetime import datetime
import math
import threading
import time
import signal
import yaml
import os

from robotino_core.Comm import Comm
from robotino_core.Graph import Graph
from robotino_core.agv.AGV_Action_ROS import Action
from robotino_core.agv.AGV_Routing import Routing
from robotino_core.agv.AGV_TaskAllocation import TaskAllocation
from robotino_core.agv.AGV_ResourceManagement import ResourceManagement
from robotino_core.solvers.astar_solver import find_shortest_path

import tf
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion


class AGV:
	"""
		A class containing the intelligence of the agv agent
	"""

	def __init__(self, ip, port, id, host, user, password, database):

		# Params
		self.ip = ip
		self.port = port
		self.host = host
		self.user = user
		self.password = password
		self.database = database

		# Open database connection
		self.comm = Comm(self.ip, self.port, self.host, self.user, self.password, self.database)
		self.comm.sql_open()

		# Read params
		this_dir = os.path.dirname(os.path.dirname(__file__))
		data_path = os.path.join(this_dir, "params", "setup2.yaml")
		with open(data_path, 'r') as file:
			self.params = yaml.load(file, Loader=yaml.FullLoader)

		# Create graph
		self.graph = Graph()
		node_neighbors = self.params['node_neighbors']
		node_locations = self.params['node_locations']
		self.graph.create_nodes(list(node_locations.values()), list(node_locations.keys()))
		self.graph.create_edges(list(node_neighbors.keys()), list(node_neighbors.values()))

		# Updated attributes
		self.id = id
		self.x_loc = 0.0
		self.y_loc = 0.0
		self.theta = 0.0
		self.node = self.search_closest_node((self.x_loc, self.y_loc))
		self.status = 'IDLE'
		self.battery_status = 100.0
		self.travelled_time = 0.0
		self.charged_time = 0.0
		self.congestions = 0
		self.task_executing = {'id': -1, 'node': None}
		self.path = []
		self.total_path = []
		
		# Dmas attributes
		self.slots = []
		self.reserved_paths = {}
		self.reserved_slots = {}

		# Init transfrom listener
		self.tf_listener = tf.TransformListener()

		# Init subscriber
		self.status_sub = rospy.Subscriber("/odom", Odometry, self.amcl_callback)

		# Depot station
		self.depot = 'pos_1'

		# Task agents
		self.task_allocation = TaskAllocation(self)
		self.routing = Routing(self)
		self.resource_management = ResourceManagement(self)
		self.action = Action(self)

		# Exit procedure
		self.exit_event = threading.Event()

		# Processes
		z = threading.Thread(target=self.task_allocation.main)
		z.daemon = True
		z.start()
		q = threading.Thread(target=self.routing.main)
		q.daemon = True
		q.start()
		#r = threading.Thread(target=self.routing.homing)
		#r.daemon = True
		#r.start()
		s = threading.Thread(target=self.resource_management.main)
		s.daemon = True
		s.start()

		self.main()
		
		# Shutdown
		rospy.on_shutdown(self.shutdown)

	def shutdown(self):

		# Closing threads
		print("\nShutdown robot")
		self.exit_event.set()
		time.sleep(1)

		# Cancel goal
		self.action.client.cancel_all_goals()

		# Open database connection
		comm = Comm(self.ip, self.port, self.host, self.user, self.password, self.database)
		comm.sql_open()

		# Delete robot from database
		comm.sql_delete_from_table('global_robot_list', 'id', self.id)
		
		# Make all tasks unassigned
		tasks = []
		local_task_list = comm.sql_get_local_task_list(self.id)
		for task in local_task_list: 
			if not task['message'] in ['homing', 'charging']:
				tasks.append((-1, 'unassigned', '-', 0, task['id']))
			else:
				tasks.append((-1, 'aborted', task['message'], 0, task['id']))
		if not self.task_executing['id'] == -1: tasks.append((-1, 'unassigned', '-', 0, self.task_executing['id']))
		if not len(tasks) == 0: comm.sql_update_tasks(tasks)

		# Close connection
		comm.sql_close()
		exit(0)

	def main(self):

		# Open database connection
		rospy.loginfo("\nAGV " + str(self.id) + ":        Started")
		comm = Comm(self.ip, self.port, self.host, self.user, self.password, self.database)
		comm.sql_open()

		# Loop
		while not rospy.is_shutdown():

			# Wait for a task on the local task list
			items = comm.sql_get_local_task_list(self.id)
			if items: 
				for row in items: self.task_executing = row; break 
			
			# Execute task
			if not self.task_executing['id'] == -1:

				# Start task
				rospy.loginfo("Agv " + str(self.id) + ":        Start executing task " + str(self.task_executing['id']))
				if self.status != 'EMPTY': self.status = 'BUSY'

				# Remove from local task list and add task to executing list
				task_dict = {'robot': self.id, 'status': 'executing', 'message': self.task_executing['message'], 'priority': self.task_executing['priority']}
				comm.sql_update_task(self.task_executing['id'], task_dict)

				# Go to task
				self.execute_task(self.task_executing)

				# Perform task
				self.action.pick()
				rospy.loginfo("Agv " + str(self.id) + ":        Picked item of task " + str(self.task_executing['id']))

				# Task executed
				task_dict = {'robot': self.id, 'status': 'done', 'message': self.task_executing['message'], 'priority': self.task_executing['priority']}
				comm.sql_update_task(self.task_executing['id'], task_dict)
				self.task_executing = {'id': -1, 'node': None}

				# Set status to IDLE when task is done or when done charging
				if self.status != 'EMPTY': self.status = 'IDLE'

	def execute_task(self, task):

		# Compute dmas path towards task destination
		self.path, _ = find_shortest_path(self.graph, self.node, task['node'])
		self.path = self.path[1:]
		print(self.path)

		# Move from node to node
		while not rospy.is_shutdown():
			
			result = self.action.move_to_node(self.path[0])
			if result: 
				rospy.loginfo("Goal execution done!")
				self.path = self.path[1:]
				#self.slots = self.slots[1:]

			if not len(self.path) > 0:
				break

			rospy.on_shutdown(self.shutdown)

	def search_closest_node(self, loc):
		node = min(self.graph.nodes.values(), key=lambda node: self.calculate_euclidean_distance(node.pos, loc))
		return node.name

	@staticmethod
	def calculate_euclidean_distance(a, b):
		return math.sqrt(math.pow(b[0] - a[0], 2) + math.pow(b[1] - a[1], 2))

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
		self.x_loc = trans[0] #msg.pose.pose.position.x
		self.y_loc = trans[1] # msg.pose.pose.position.y

		# Get orientation
		#orientation_q = msg.pose.pose.orientation
		#orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		#(_, _, yaw) = euler_from_quaternion(rot)
		self.theta = rotation[2] #yaw

		# Search closest node
		self.node = self.search_closest_node((self.x_loc, self.y_loc))

		# Update robot
		self.update_global_robot_list()

	def update_global_robot_list(self):
		robot_dict = {"id": self.id, "ip": self.ip, "port": self.port, "x_loc": self.x_loc, "y_loc": self.y_loc, "theta": self.theta, "node": self.node,
				"status": self.status, "battery_status": self.battery_status, "travelled_time": self.travelled_time, "charged_time": self.charged_time,
				"congestions": self.congestions, "task_executing": self.task_executing['id'], "path": str(self.path), "total_path": str(self.total_path)}
		self.comm.sql_add_to_table('global_robot_list', robot_dict)
		self.comm.sql_update_robot(self.id, robot_dict)
