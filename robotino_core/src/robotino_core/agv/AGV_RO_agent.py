import threading
import sys
from datetime import datetime, timedelta
sys.setrecursionlimit(10000)
import os
import yaml

from robotino_core.Comm import Comm
from robotino_core.solvers.astar_solver import *
from robotino_core.solvers.dmas_solver import *


class AGV_RO_agent:

	"""
			A class containing the intelligence of the Routing agent. This agent takes care of real-time route updates based on the 
			tasks currently assinged to the robot, and based on the status of the layout reservations.
	"""

	def __init__(self, params_file):

		# Read params
		this_dir = os.path.dirname(os.path.dirname(__file__))
		data_path = os.path.join(this_dir, "params", params_file)
		with open(data_path, 'r') as file:
			self.params = yaml.load(file, Loader=yaml.FullLoader)

		# Init database communication for all threads
		self.comm_route_planner = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_route_evaporator = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		
		# Exit event
		self.exit_event = threading.Event()

		# Processes
		self.threads = []
		self.threads.append(threading.Thread(target=self.route_planner))
		self.threads.append(threading.Thread(target=self.route_evaporator))

	def signal_handler(self, _, __):
		print("\n\nShutdown RO-agent")
		self.exit_event.set()

	def route_planner(self):
		
		"""
		This thread continuously checks all tasks on the local task list and plans a route through these tasks.
		 It first plans a route to the task that is currently executing and further plans a route through all tasks assinged.

		"""

		# Loop
		print("\nRO-agent:	Route planner thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=self.params['route_planner_rate']): 
				print("\nRO-agent:	Route planner thread killed")
				break

			# Get graph
			graph = self.comm_route_planner.sql_select_graph()
			if graph is None: continue

			# Get robot
			robot = self.comm_route_planner.sql_select_robot(self.params['id'])
			if robot is None or len(robot) == 0: continue

			# Get task executing
			task_executing = self.comm_route_planner.sql_select_executing_task(robot[0]['id'])
			if task_executing is None or len(task_executing) == 0: continue

			# Plan and evaluate path and slots towards executing task
			print("\nRO-agent:	Plan executing task")
			task_executing_end_time = self.plan_executing_task(graph, robot[0], task_executing[0])
			if not task_executing_end_time: continue

			# Plan and evaluate path and slots towards local tasks
			print("RO-agent:	Plan local task list")
			self.plan_local_tasks(graph, robot[0], task_executing[0], task_executing_end_time)

	def route_evaporator(self):

		# Loop
		print("RO-agent:	Route evaporator thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=self.params['route_evaporator_rate']): 
				print("\nRO-agent:	Route evaporator thread started")
				break

			# Get all reservations of this robot
			reservations = self.comm_route_evaporator.sql_select_robot_reservations(self.params['id'])
			if reservations is None: continue

			# Evaporate all reservations
			pheromones = []
			for reservation in reservations:
				pheromone = max(self.params['t0'], reservation['pheromone'] * (1 - self.params['rho']))
				if pheromone <= self.params['t0']: pheromone = -1 
				pheromones.append((pheromone, reservation['id']))
			self.comm_route_evaporator.sql_update_reservations(pheromones)
			self.comm_route_evaporator.sql_delete_from_table('environmental_agents' , 'pheromone', -1)

	def plan_executing_task(self, graph, robot, task_executing):

		"""
		This function plans a route to the executing task, starting from the node the robot is currently driving to.

		"""

		# Calculate dist and cost to next node
		dist_to_next = self.dist_euclidean((robot['x_loc'], robot['y_loc']), graph.nodes[robot['next_node']].pos)
		cost_to_next = dist_to_next / robot['speed']

		# Start situation
		start_node = robot['next_node']
		start_time = datetime.now() + timedelta(seconds=cost_to_next)

		# Do dmas towards executing task
		path, slots, dist, cost = dmas(start_node, [task_executing['node']], start_time, graph, robot['id'], robot['speed'], self.comm_route_planner)

		# Add charging time 
		if task_executing['message'] == 'charging': slots[-1] = (slots[-1][0], slots[-1][1] + timedelta(seconds=0)) # TODO add charging time

		# Reserve slots
		res = intent(path, slots, robot['id'], self.comm_route_planner)
		if not res: return False

		# Compute executing task params
		task_executing_end_time = slots[-1][0] + slots[-1][1]
		duration = (task_executing_end_time - start_time).total_seconds()

		# Update executing task
		task_dict = {'estimated_end_time': task_executing_end_time.strftime('%H:%M:%S'), 'estimated_duration': str(duration), 'path': str(path), 'slots':str(slots), 'dist': dist, 'cost': cost}
		self.comm_route_planner.sql_update_task(task_executing['id'], task_dict)

		# Return 
		return task_executing_end_time

	def plan_local_tasks(self, graph, robot, task_executing, task_executing_end_time):

		# Get tasks in local task list
		local_task_list = self.comm_route_planner.sql_select_local_task_list(robot['id'])
		if local_task_list is None or len(local_task_list) == 0: return False

		# Start situation
		start_node = task_executing['node']
		start_time = task_executing_end_time

		# Plan routes for all tasks
		priority = 1
		for task in local_task_list:

			# Do dmas towards task
			path, slots, dist, cost = dmas(start_node, [task['node']], start_time, graph, robot['id'], robot['speed'], self.comm_route_planner)

			# Add charging time 
			if task_executing['message'] == 'charging': slots[-1] = (slots[-1][0], slots[-1][1] + timedelta(seconds=0)) # TODO add charging time

			# Reserve slots
			res = intent(path, slots, robot['id'], self.comm_route_planner)
			if not res: return False

			# Compute task params
			end_time = slots[-1][0] + slots[-1][1]
			duration = (end_time - start_time).total_seconds()

			# Update task
			task_dict = {'priority': priority, 'estimated_start_time': start_time.strftime('%H:%M:%S'), 'estimated_end_time': end_time.strftime('%H:%M:%S'), 'estimated_duration': str(duration), 'path': str(path), 'slots':str(slots), 'dist': dist, 'cost': cost}
			self.comm_route_planner.sql_update_task(task['id'], task_dict)		

			# Update state
			start_node = task['node']
			start_time = end_time
			priority += 1

	@staticmethod
	def dist_euclidean(a, b):
		return math.sqrt(math.pow(b[0] - a[0], 2) + math.pow(b[1] - a[1], 2))

