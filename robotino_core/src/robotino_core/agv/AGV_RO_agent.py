import sys
from datetime import datetime, timedelta
sys.setrecursionlimit(10000)
import os
import yaml
import threading

from robotino_core.Comm import Comm
from robotino_core.solvers.astar_solver import *
from robotino_core.solvers.tsp_solver import *
from robotino_core.solvers.dmas_solver import *


class AGV_RO_agent:

	"""
			A class containing the intelligence of the Routing agent
	"""

	def __init__(self, params_file):

		# Read params
		this_dir = os.path.dirname(os.path.dirname(__file__))
		data_path = os.path.join(this_dir, "params", params_file)
		with open(data_path, 'r') as file:
			self.params = yaml.load(file, Loader=yaml.FullLoader)

		# Init database communication
		self.comm_main = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_main.sql_open()

		# Attribute
		self.task_executing_end_time = 0

		# Exit event
		self.exit_event = threading.Event()

		# Processes
		self.threads = []
		self.threads.append(threading.Thread(target=self.main))

	def signal_handler(self, _, __):
		print("\n\nShutdown RO robot without communication possibilities")
		self.exit_event.set()

	def main(self):
		
		# Loop
		print("RO-agent:	Main thread started")
		while True:

			# Plan and evaluate path and slots towards executing task
			self.plan_executing_task()

			# Plan and evaluate path and slots towards local tasks
			self.plan_local_tasks()

			# Timeout and exit event
			if self.exit_event.wait(timeout=self.params['ro_rate']): 
				print("RO-agent:	Main thread killed")
				break

	def plan_executing_task(self):

		### Inputs ###

		# Get robot
		robot = self.comm_main.sql_get_robot(self.params['id'])
		if robot is None or len(robot) == 0: return False
		robot = robot[0]

		# Get task executing
		task_executing = self.comm_main.sql_get_executing_task(robot['id'])
		if task_executing is None or len(task_executing) == 0: return False
		task_executing = task_executing[0]

		# Get graph
		graph = self.comm_main.sql_get_graph()
		if graph is None: return False

		### Computation ###

		# Calculate dist and cost to next node
		dist_to_next = self.dist_euclidean((robot['x_loc'], robot['y_loc']), graph.nodes[robot['next_node']].pos)
		cost_to_next = dist_to_next / robot['speed']

		# Start situation
		start_node = robot['next_node']
		start_time = datetime.now() + timedelta(seconds=cost_to_next)

		# Do dmas untill successful reservation
		while True:

			# Do dmas towards executing task
			path, slots, dist, cost = dmas(start_node, [task_executing['node']], start_time, graph, robot['id'], robot['speed'], self.comm_main)

			# Add charging time 
			if task_executing['message'] == 'charging':
				slots[-1] = (slots[-1][0], slots[-1][1] + timedelta(seconds=0)) # TODO add charging time

			# Reserve slots
			res = intent(path, slots, robot['id'], self.comm_main)

			# End criterium
			if res: break

		### Outputs ###

		# Update executing task
		self.task_executing_end_time = slots[-1][0] + slots[-1][1]
		duration = (self.task_executing_end_time - start_time).total_seconds()
		task_dict = {'estimated_end_time': self.task_executing_end_time.strftime('%H:%M:%S'), 'estimated_duration': str(duration), 'path': str(path), 'slots':str(slots), 'dist': dist, 'cost': cost}
		res = self.comm_main.sql_update_task(task_executing['id'], task_dict)

	def plan_local_tasks(self):

		### Inputs ###

		# Get robot
		robot = self.comm_main.sql_get_robot(self.params['id'])
		if robot is None or len(robot) == 0: return False
		robot = robot[0]

		# Get task executing
		task_executing = self.comm_main.sql_get_executing_task(robot['id'])
		if task_executing is None or len(task_executing) == 0: return False
		task_executing = task_executing[0]

		# Get graph
		self.graph = self.comm_main.sql_get_graph()
		if self.graph is None: return False

		# Get tasks in local task list
		local_task_list = self.comm_main.sql_get_local_task_list(robot['id'])
		if local_task_list is None or len(local_task_list) == 0: return False

		### Computation ###

		# Start situation
		start_node = task_executing['node']
		start_time = self.task_executing_end_time

		# Extract only node names from tasks without homing task or charging task
		# homing = [task for task in local_task_list if task['message'] == 'homing']
		# charging = [task for task in local_task_list if task['message'] == 'charging']
		# nodes_to_visit = [task['node'] for task in local_task_list if not task['message'] == 'homing' and not task['message'] == 'charging']

		# Do tsp using astar routing
		# solution = tsp(start_node, nodes_to_visit, self.dist_astar)

		# Convert task node names back to tasks including homing task
		# sequence = [local_task_list[nodes_to_visit.index(name)] for name in solution['sequence']] + homing 

		# Plan routes for all tasks
		priority = 1
		for task in local_task_list:

			# Do dmas untill successful reservation
			while True:

				# Update local_task_list
				path, slots, dist, cost = dmas(start_node, [task['node']], start_time, self.graph, robot['id'], robot['speed'], self.comm_main)

				# Add charging time 
				if task_executing['message'] == 'charging':
					slots[-1] = (slots[-1][0], slots[-1][1] + timedelta(seconds=0)) # TODO add charging time

				# Reserve slots
				res = intent(path, slots, robot['id'], self.comm_main)

				# End criterium
				if res: break

			### Outputs ###

			# Update task
			end_time = slots[-1][0] + slots[-1][1]
			duration = (end_time - start_time).total_seconds()
			task_dict = {'priority': priority, 'estimated_start_time': start_time.strftime('%H:%M:%S'), 'estimated_end_time': end_time.strftime('%H:%M:%S'), 'estimated_duration': str(duration), 'path': str(path), 'slots':str(slots), 'dist': dist, 'cost': cost}
			res = self.comm_main.sql_update_task(task['id'], task_dict)		

			# Update state
			start_node = task['node']
			start_time = end_time
			priority += 1	

	def dist_astar(self, a, b):
		assert isinstance(a, str)
		assert isinstance(b, str)
		return get_shortest_path(self.graph, a, b)

	@staticmethod
	def dist_euclidean(a, b):
		return math.sqrt(math.pow(b[0] - a[0], 2) + math.pow(b[1] - a[1], 2))

