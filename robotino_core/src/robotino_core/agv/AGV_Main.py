from datetime import datetime
import math
import threading
import time
import signal
import yaml
import os

from robotino_core.Comm import Comm
from robotino_core.agv.AGV_Action import Action
from robotino_core.agv.AGV_Routing import Routing
from robotino_core.agv.AGV_TaskAllocation import TaskAllocation
from robotino_core.agv.AGV_ResourceManagement import ResourceManagement
from robotino_core.solvers.astar_solver import find_shortest_path

class AGV:

	"""
		A class containing the intelligence of the agv agent
	"""

	def __init__(self, ip, port, id, host, user, password, database, loc, ta_agent, ro_agent, rm_agent):

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
		self.comm = Comm(self.ip, self.port, self.host, self.user, self.password, self.database)
		self.comm.sql_open()
		self.graph = self.comm.get_graph()
		
		# Updated attributes
		self.id = id
		self.x_loc = loc[0]
		self.y_loc = loc[1]
		self.theta = 0.0
		self.node = self.search_closest_node(loc)
		self.status = 'IDLE'
		self.battery_status = 100.0
		self.travelled_time = 0.0
		self.charged_time = 0.0
		self.congestions = 0

		# Attribute of current executing task
		self.task_executing = {'id': -1, 'node': None, 'message': '-'}
		self.dist_to_do = 0.0 # Current distance to execute 
		self.dist_done = 0.0 # Current distance already executed
		self.start_time = None # Start time of executing task
		self.estimated_end_time = None # Estimated end time of executing task

		# Attributes of planned path through all assigned tasks
		self.current_node = self.node # Current node moving to
		self.current_path = [] # Current path following
		self.total_path = [] # Total planned path 
		self.reserved_paths = {} # All planned paths for tasks in local task list
		self.reserved_slots = {} # All reserved slots for tasks in local task list

		# Depot station
		self.depot = self.search_closest_node(loc)

		# Task agents
		self.ta_agent = None
		self.ro_agent = None
		self.rm_agent = None		
		self.action = Action(self)

		# Exit procedure
		self.exit_event = threading.Event()
		signal.signal(signal.SIGINT, self.signal_handler)

		# Main AGV agent processes
		threads = []
		threads.append(threading.Thread(target=self.main))
		threads.append(threading.Thread(target=self.odom_callback))

		# Task allocation agent processes
		if ta_agent:
			self.ta_agent = TaskAllocation(self)
			threads.append(threading.Thread(target=self.ta_agent.main))
			threads.append(threading.Thread(target=self.ta_agent.progress_tracker))
			threads.append(threading.Thread(target=self.ta_agent.local_consensus))

		# Routing agent processes
		if ro_agent:
			self.ro_agent = Routing(self)
			threads.append(threading.Thread(target=self.ro_agent.main))
			threads.append(threading.Thread(target=self.ro_agent.homing))

		# Resource management agent processes
		if rm_agent:
			self.rm_agent = ResourceManagement(self)
			threads.append(threading.Thread(target=self.rm_agent.main))
		
		# Start processes
		for thread in threads:
			thread.daemon = True
			thread.start()

		# Join processes
		for thread in threads:
			thread.join()

	def main(self):

		# Open database connection
		print("\nAGV " + str(self.id) + ":                       Started")

		# Loop
		while True:

			# Wait for a task on the local task list and pick first (lowest priority)
			for row in self.comm.sql_get_local_task_list(self.id): self.task_executing = row; break 

			# Execute task if exist
			if not self.task_executing['id'] == -1:
				
				# Start task
				print("\nAgv " + str(self.id) + ":        Start executing task " + str(self.task_executing['id']))
				if self.status != 'EMPTY': self.status = 'BUSY'

				# Remove from local task list and add task to executing list
				self.start_time = datetime.now()
				task_dict = {'robot': self.id, 'status': 'executing', 'real_start_time': self.start_time.strftime('%H:%M:%S')}
				self.comm.sql_update_task(self.task_executing['id'], task_dict)

				# Move to task
				result = self.execute_task(self.task_executing)

				# If task reached
				if result:

					# Pick task
					if not self.task_executing['message'] not in ['homing', 'charging']: self.action.pick()

					# Task executed
					end_time = datetime.now()
					duration = end_time - self.start_time
					task_dict = {'status': 'done', 'real_end_time': end_time.strftime('%H:%M:%S'), 'real_duration': str(duration)}
					self.comm.sql_update_task(self.task_executing['id'], task_dict)
					self.task_executing = {'id': -1, 'node': None}

				# If task not reached
				else:

					# Make task unnasigned if not able to reach
					task_dict = {'status': 'unassigned', 'message': '-', 'priority': 0}
					self.comm.sql_update_task(self.task_executing['id'], task_dict)
					self.task_executing = {'id': -1, 'node': None}

				# Set status to IDLE when task is done or when done charging
				if self.status != 'EMPTY': self.status = 'IDLE'

			# Close thread at close event 
			if self.exit_event.is_set():
				break

	def execute_task(self, task):

		# Compute path towards task destination
		if self.ro_agent:
			self.dist_to_do = 0.0
			if not task['id'] in self.reserved_paths.keys():
				self.ro_agent.plan_executing_task(self.comm)
		else:
			path, dist = find_shortest_path(self.comm.get_graph(), self.node, task['node'])
			self.dist_to_do = dist
			self.reserved_paths[task['id']] = path
			self.reserved_slots[task['id']] = [0]

		# Move from node to node
		self.dist_done = 0.0 
		while not self.node == task['node']:

			# Set current path
			self.current_path = self.reserved_paths[task['id']]

			# Current node moving to
			self.current_node = self.reserved_paths[task['id']][0]
			self.current_slot = self.reserved_slots[task['id']][0]

			# If routing agent
			if self.ro_agent:

				# Compute path towards task destination
				if self.reserved_paths[task['id']][0] == self.node:
					self.ro_agent.plan_executing_task(self.comm)

				# Compute total distance to travel
				self.dist_to_do = self.dist_done + sum([self.graph.edges[self.reserved_paths[task['id']][i], self.reserved_paths[task['id']][i + 1]].length for i in range(len(self.reserved_paths[task['id']]) - 1)])

				# Wait for node to be free
				node_arriving_time = self.current_slot[0]
				if node_arriving_time > datetime.now():
					td = node_arriving_time - datetime.now()
					time.sleep(td.total_seconds())
					print("Agv " + str(self.id) + ":        Waits " + str(td) + " seconds")

			# If no routing agent
			else:
				self.reserved_paths[task['id']] = self.reserved_paths[task['id']][1:]
					
			# Move to node if free			
			result = self.action.move_to_node(self.current_node)
			if not result: return False

		# Check if task is charging task
		if task['message'] == 'charging':

			# Update status
			print("AGV " + str(self.id) + ":        Is charging for " + str(5) + " seconds")
			self.status = 'CHARGING'

			# Charging
			time.sleep(5)

			# Update robot status
			self.battery_status = 100
			self.status = 'IDLE'

		return True

	def odom_callback(self):

		# Open database connection
		comm = Comm(self.ip, self.port, self.host, self.user, self.password, self.database)
		comm.sql_open()

		# Loop
		while True:

			# Update robot
			self.update_global_robot_list(comm)

			# Close thread at close event 
			if self.exit_event.is_set():
				break

	def update_global_robot_list(self, comm):
		robot_dict = {"id": self.id, "ip": self.ip, "port": self.port, "x_loc": self.x_loc, "y_loc": self.y_loc, "theta": self.theta, "node": self.node,
				"status": self.status, "battery_status": self.battery_status, "travelled_time": self.travelled_time, "charged_time": self.charged_time,
				"congestions": self.congestions, "task_executing": self.task_executing['id'], "moving_to": self.current_node, "path": str(self.current_path), "total_path": str(self.total_path), "time_now": datetime.now().strftime('%H:%M:%S')}
		comm.sql_add_to_table('global_robot_list', robot_dict)
		comm.sql_update_robot(self.id, robot_dict)

	def search_closest_node(self, loc):
		graph = self.comm.get_graph()
		node = min(graph.nodes.values(), key=lambda node: self.calculate_euclidean_distance(node.pos, loc))
		return node.name

	@staticmethod
	def calculate_euclidean_distance(a, b):
		return math.sqrt(math.pow(b[0] - a[0], 2) + math.pow(b[1] - a[1], 2))

	def signal_handler(self, _, __):

		# Close threads
		print("\nShutdown robot without communication possibilities")
		self.exit_event.set()
		time.sleep(1)
		exit(0)
