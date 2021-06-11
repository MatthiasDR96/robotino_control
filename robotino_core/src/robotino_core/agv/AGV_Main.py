from datetime import datetime
import math
import threading
import time
import signal
import yaml
import os

from robotino_core.Comm import Comm
from robotino_core.Graph import Graph
from robotino_core.agv.AGV_Action import Action
from robotino_core.agv.AGV_Routing import Routing
from robotino_core.agv.AGV_TaskAllocation import TaskAllocation
from robotino_core.agv.AGV_ResourceManagement import ResourceManagement
from robotino_core.solvers.astar_solver import find_shortest_path

class AGV:
	"""
		A class containing the intelligence of the agv agent
	"""

	def __init__(self, ip, port, id, host, user, password, database, loc):

		# Params
		self.ip = ip
		self.port = port
		self.host = host
		self.user = user
		self.password = password
		self.database = database

		# Read params
		this_dir = os.path.dirname(os.path.dirname(__file__))
		data_path = os.path.join(this_dir, "params", "setup.yaml")
		with open(data_path, 'r') as file:
			self.params = yaml.load(file, Loader=yaml.FullLoader)

		# Create graph
		self.graph = Graph()
		self.node_names = self.params['node_names']
		self.node_locations = self.params['node_locations']
		self.graph.create_nodes(self.node_locations, list(self.node_names.keys()))
		self.graph.create_edges(list(self.node_names.keys()), list(self.node_names.values()))

		# Updated attributes
		self.id = id
		self.x_loc = loc[0]
		self.y_loc = loc[1]
		self.theta = 0.0
		self.node = self.search_closest_node(loc)
		self.status = 'IDLE'
		self.battery_status = 40.0
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

		# Depot station
		self.depot = self.search_closest_node(loc)

		# Task agents
		self.task_allocation = TaskAllocation(self)
		self.routing = Routing(self)
		self.resource_management = ResourceManagement(self)
		self.action = Action(self)

		# Exit procedure
		self.exit_event = threading.Event()
		signal.signal(signal.SIGINT, self.signal_handler)

		# Processes
		x = threading.Thread(target=self.odom_callback)
		x.daemon = True
		x.start()
		y = threading.Thread(target=self.main)
		y.daemon = True
		y.start()
		z = threading.Thread(target=self.task_allocation.main)
		z.daemon = True
		z.start()
		q = threading.Thread(target=self.routing.main)
		q.daemon = True
		q.start()
		r = threading.Thread(target=self.routing.homing)
		r.daemon = True
		r.start()
		s = threading.Thread(target=self.resource_management.main)
		s.daemon = True
		s.start()
		x.join()
		y.join()
		z.join()
		q.join()
		r.join()
		s.join()

	def main(self):

		# Open database connection
		print("\nAGV " + str(self.id) + ":                       Started")
		comm = Comm(self.ip, self.port, self.host, self.user, self.password, self.database)
		comm.sql_open()

		# Loop
		while True:

			# Wait for a task on the local task list
			items = comm.sql_get_local_task_list(self.id)
			if items: 
				for row in items: self.task_executing = row; break 

			# Execute task
			if not self.task_executing['id'] == -1:
				
				# Start task
				print("Agv " + str(self.id) + ":        Start executing task " + str(self.task_executing['id']))
				if self.status != 'EMPTY': self.status = 'BUSY'

				# Remove from local task list and add task to executing list
				task_dict = {'robot': self.id, 'status': 'executing', 'message': self.task_executing['message'], 'priority': self.task_executing['priority']}
				comm.sql_update_task(self.task_executing['id'], task_dict)

				# Go to task
				self.execute_task(self.task_executing)

				# Perform task
				self.action.pick()
				print("Agv " + str(self.id) + ":        Picked item of task " + str(self.task_executing['id']))

				# Task executed
				task_dict = {'robot': self.id, 'status': 'done', 'message': self.task_executing['message'], 'priority': self.task_executing['priority']}
				comm.sql_update_task(self.task_executing['id'], task_dict)
				self.task_executing = {'id': -1, 'node': None}

				# Set status to IDLE when task is done or when done charging
				if self.status != 'EMPTY': self.status = 'IDLE'

			# Close thread at close event 
			if self.exit_event.is_set():
				break

	def execute_task(self, task):

		# Compute path towards task destination
		print("Move to " + str(task['node']))
		if self.params['routing'] == True:
			if task['id'] in self.reserved_paths.keys():
				self.path = self.reserved_paths[task['id']]
				self.slots = self.reserved_slots[task['id']]
			else:
				self.path, self.slots, _ = self.routing.dmas(self.node, [task['node']], datetime.now())
		else:
			self.path, _ = find_shortest_path(self.graph, self.node, task['node'])

		# Move from node to node
		while len(self.path) > 0:
			# Wait for node to be free
			# if self.params['routing'] == True:
				# node_arriving_time = self.slots[0][0]
				# if node_arriving_time > datetime.now():
					# td = node_arriving_time - datetime.now()
					# time.sleep(td.total_seconds())

			# Move to node if free
			self.action.move_to_node(self.path[0])

		# Check if task is charging task
		if task['message'] == 'charging':

			# Update status
			print("AGV " + str(self.id) + ":      Is charging for " + str(5) + " seconds")
			self.status = 'CHARGING'

			# Charging
			time.sleep(5)

			# Update robot status
			self.battery_status = 100
			self.status = 'IDLE'

	def search_closest_node(self, loc):
		node = min(self.graph.nodes.values(), key=lambda node: self.calculate_euclidean_distance(node.pos, loc))
		return node.name

	@staticmethod
	def calculate_euclidean_distance(a, b):
		return math.sqrt(math.pow(b[0] - a[0], 2) + math.pow(b[1] - a[1], 2))

	def odom_callback(self):

		# Open database connection
		comm = Comm(self.ip, self.port, self.host, self.user, self.password, self.database)
		comm.sql_open()

		while True:

			# Update robot
			self.update_global_robot_list(comm)

			# Close thread at close event 
			if self.exit_event.is_set():
				break

	def update_global_robot_list(self, comm):
		robot_dict = {"id": self.id, "ip": self.ip, "port": self.port, "x_loc": self.x_loc, "y_loc": self.y_loc, "theta": self.theta, "node": self.node,
				"status": self.status, "battery_status": self.battery_status, "travelled_time": self.travelled_time, "charged_time": self.charged_time,
				"congestions": self.congestions, "task_executing": self.task_executing['id'], "path": str(self.path), "total_path": str(self.total_path)}
		comm.sql_add_to_table('global_robot_list', robot_dict)
		comm.sql_update_robot(self.id, robot_dict)

	def signal_handler(self, _, __):

		# Close threads
		print("\nShutdown robot")
		self.exit_event.set()
		time.sleep(1)
		
		# Open database connection
		comm = Comm(self.ip, self.port, self.host, self.user, self.password, self.database)
		comm.sql_open()

		# Delete robot from database
		comm.sql_delete_from_table('global_robot_list', 'id', self.id)
		
		# Make all tasks unassigned
		tasks = []
		local_task_list = comm.sql_get_local_task_list(self.id)
		for task in local_task_list: tasks.append((-1, 'unassigned', '', 0, task['id']))
		if not self.task_executing['id'] == -1: tasks.append((-1, 'unassigned', '', 0, self.task_executing['id']))
		if not len(tasks) == 0: comm.sql_update_tasks(tasks)

		# Close connection
		comm.sql_close()
		exit(0)
