import sys
from datetime import datetime, timedelta
sys.setrecursionlimit(10000)
import threading
import yaml
import os
import time
import math
import ast

from robotino_core.Comm import Comm
from robotino_core.agv.AGV_Action import AGV_Action
from robotino_core.solvers.astar_solver import *


class AGV_Main:

	"""
		A class containing the intelligence of the main AGV-agent
	"""

	def __init__(self, params_file):

		# Read params
		this_dir = os.path.dirname(os.path.dirname(__file__))
		data_path = os.path.join(this_dir, "params", params_file)
		with open(data_path, 'r') as file:
			self.params = yaml.load(file, Loader=yaml.FullLoader)
		
		# Init database communication for all threads
		self.comm_task_executer = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_status_updater = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_alive_checker = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_progress_tracker = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_performance_monitor = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		
		# Action layer		
		self.action = AGV_Action(self)

		# Depot station
		self.depot = self.params['depot'] #random.choice(list(self.comm_task_executer.sql_select_graph().nodes.keys()))

		# Current attributes - status at the moment
		self.id = self.params['id']
		self.x_loc = self.comm_task_executer.sql_select_graph().nodes[self.depot].pos[0]
		self.y_loc = self.comm_task_executer.sql_select_graph().nodes[self.depot].pos[1]
		self.theta = 0.0
		self.speed = self.params['robot_speed']
		self.node = self.depot
		self.status = 'IDLE'
		self.battery_status = 100.0

		# History attributes - work done
		self.start_time = datetime.now()
		self.sql_queries = 0
		self.traveled_dist = 0.0
		self.traveled_cost = 0.0
		self.charged_time = 0.0
		self.congestions = 0

		# Executing task attributes
		self.task_executing_dist = 0.0 # Estimated dist to do for this task
		self.task_executing_cost = 0.0 # Estimated cost to do for this task
		self.task_executing_dist_done = 0.0 # Total dist traveled for this task
		self.task_executing_total_dist = 0.0 # Total dist to travel for this task
		self.local_task_list_dist = 0.0 # Estimated dist to do for all tasks in local task list
		self.local_task_list_cost = 0.0 # Estimated cost to do for all tasks in local task list
		self.next_node = self.node # Current node moving to
		self.next_slot = [] # Current slot moving to
		self.current_path = [] # Current path following
		self.total_path = [] # Total path through all tasksin local task list
		
		# Create logger table for performance in time
		self.comm_task_executer.sql_drop_table('robot' + str(self.id))
		self.comm_task_executer.sql_create_robot_table('robot' + str(self.id))

		# Exit event
		self.exit_event = threading.Event()

		# Processes
		self.threads = []
		self.threads.append(threading.Thread(target=self.task_executer))
		self.threads.append(threading.Thread(target=self.status_updater))
		self.threads.append(threading.Thread(target=self.alive_checker))
		self.threads.append(threading.Thread(target=self.progress_tracker))
		self.threads.append(threading.Thread(target=self.performance_monitor))
		
	def task_executer(self):

		"""
		This thread executes the first task on the local task list (with priority one).

		"""

		# Loop
		print("\nAGV-agent:	Task executer thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=1): 
				print("\nAGV-agent:	Task executer thread killed")
				break

			# Wait for a task on the local task list 
			local_task_list = self.comm_task_executer.sql_select_local_task_list(self.id)
			if local_task_list is None or len(local_task_list) == 0: continue

			# Pick first task (lowest priority)
			task_executing = local_task_list[0]

			# Start task
			print("\nAGV-agent:	Start executing task " + str(task_executing['id']))
			if self.status != 'EMPTY': self.status = 'BUSY'

			# Do not execute
			#exit()

			# Remove from local task list and add task to executing list
			task_executing_start_time = datetime.now()
			self.task_executing_total_dist = get_shortest_path(self.comm_task_executer.sql_select_graph(), self.node, task_executing['node'])[1]
			task_dict = {'priority': 0, 'status': 'executing', 'estimated_start_time': task_executing_start_time.strftime('%H:%M:%S'), 'real_start_time': task_executing_start_time.strftime('%H:%M:%S')}
			self.comm_task_executer.sql_update_task(task_executing['id'], task_dict)

			# Move to task
			result = self.execute_task(task_executing)

			# If task reached
			if result:

				# Pick task
				if not task_executing['message'] in ['homing', 'charging']: self.action.pick()

				# Task executed
				end_time = datetime.now()
				duration = (end_time - task_executing_start_time).total_seconds()
				task_dict = {'status': 'done', 'progress': 100, 'real_end_time': end_time.strftime('%H:%M:%S'), 'real_duration': str(duration)}
				self.comm_task_executer.sql_update_task(task_executing['id'], task_dict)

				# Reset executing task
				print("AGV-agent:	Finished task " + str(task_executing['id']))
					
			# If task not reached
			else:

				# Make task unnasigned if not able to reach
				task_dict = {'status': 'unassigned', 'message': '-', 'priority': 0}
				self.comm_task_executer.sql_update_task(task_executing['id'], task_dict)

				# Reset executing task
				print("AGV-agent:	Failed task " + str(task_executing['id']))
					
			# Set status to IDLE when task is done or when done charging
			if self.status != 'EMPTY': self.status = 'IDLE'

	def execute_task(self, task):

		"""
		This function executes a task and follows the suggested route if this exist. Otherwise it calculates an astar path.

		"""

		# Move from node to node until end node reached
		self.task_executing_dist_done = 0
		
		while not self.node == task['node']:

			# Get executing task
			executing_task = self.comm_task_executer.sql_select_executing_task(self.id)
			if executing_task is None or len(executing_task) == 0: return False

			# Get planned path
			task_executing_path = ast.literal_eval(executing_task[0]['path'])
			task_executing_slots = [[datetime.now(), timedelta()]] # TODO add real slot

			# Plan path if no path available
			if len(task_executing_path) == 0:
				print("AGV-agent:	Needs to plan astar")
				task_executing_path = get_shortest_path(self.comm_task_executer.sql_select_graph(), self.node, task['node'])[0][1:]
				task_executing_slots = [[datetime.now(), timedelta()]]

			# Set current path and current node moving to
			self.next_node = task_executing_path[0]
			self.next_slot = task_executing_slots[0]

			# Wait for node to be free
			node_arriving_time = self.next_slot[0]
			if node_arriving_time > datetime.now():
				td = node_arriving_time - datetime.now()
				time.sleep(td.total_seconds())
				print("Agv " + str(self.id) + ":        Waits " + str(td) + " seconds")

			# Move to node if free	
			node_position = self.comm_task_executer.sql_select_graph().nodes[self.next_node].pos	
			if not self.action.move_to_pos(node_position): return False
			self.node = self.next_node

		# Check if task is charging task
		if task['message'] == 'charging':

			# Update status
			print("AGV " + str(self.id) + ":         Is charging for " + str(5) + " seconds")
			self.status = 'CHARGING'

			# Charging
			time.sleep(5)

			# Update robot status
			self.battery_status = 100
			self.status = 'IDLE'

		return True

	def status_updater(self):

		"""
		This thread continuously updates the state of the robot on the database.

		"""

		# Loop
		print("AGV-agent:	Status updater started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=0.1): 
				print("AGV-agent:	Status updater killed")
				break

			# Make robot dict
			robot_dict = {"id": self.id, "ip": self.params['ip'], "port": self.params['port'], "x_loc": float(self.x_loc), "y_loc": float(self.y_loc), "theta": float(self.theta), "speed": float(self.speed), "node": self.node,
				"status": self.status, "battery_status": float(self.battery_status), "sql_queries": self.sql_queries, "traveled_cost": float(self.traveled_cost), "traveled_dist": float(self.traveled_dist), "charged_time": float(self.charged_time),
				"congestions": self.congestions, "next_node": self.next_node, "current_path": str(self.current_path), "total_path": str(self.total_path), 
				"task_executing_dist": float(self.task_executing_dist), "task_executing_cost": float(self.task_executing_cost), "local_task_list_dist": float(self.local_task_list_dist), "local_task_list_cost": float(self.local_task_list_cost), "time_now": datetime.now().strftime('%H:%M:%S')}

			# Update robot
			self.comm_status_updater.sql_add_to_table('global_robot_list', robot_dict)
			self.comm_status_updater.sql_update_robot(self.id, robot_dict)

			# Log data
			for key in {'id', 'ip', 'port'}: del robot_dict[key]
			self.comm_status_updater.sql_add_to_table('robot' + str(self.id), robot_dict)

			# Update graph
			self.graph = self.comm_status_updater.sql_select_graph()

	def alive_checker(self):

		"""
		This thread continuously checks if robots are still alive. It puts all tasks of a dead robot for auction.

		"""

		# Loop
		print("AGV-agent:	Alive checker thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=0.1): 
				print("AGV-agent:	Alive checker thread killed")
				break

			# Get all robots
			robots = self.comm_alive_checker.sql_select_everything_from_table('global_robot_list')
			if robots is None or len(robots) == 0: continue
				
			# Loop over all robots
			for robot in robots:

				# Compute time difference
				robot_time = datetime.strptime(robot['time_now'], '%H:%M:%S').time()
				now = (datetime.now() - timedelta(seconds = self.params['dead_time'])).time()

				# Delete if time differs too much
				if robot_time < now:

					# Print robot falldown detection
					print("AGV " + str(self.id) + " :   	 	Detected fallout of robot " + str(robot['id']))

					# Delete robot from database
					self.comm_alive_checker.sql_delete_from_table('global_robot_list', 'id', robot['id'])
						
					# Make all tasks unassigned
					local_task_list = self.comm_alive_checker.sql_select_local_task_list(robot['id'])
					task_executing = self.comm_alive_checker.sql_select_executing_task(robot['id'])
					for task in local_task_list + task_executing: 
						if not task['message'] in ['homing', 'charging']:
							task_dict = {'priority': 0, 'robot': -1, 'estimated_start_time': '-', 'estimated_end_time': '-', 'estimated_duration': '-', 'real_start_time': '-', 'real_end_time': '-', 'real_duration': '-', 'progress': 0.0, 'message': '-', 'status': 'unassigned', 'approach': task['approach'], "auctioneer": -1, 'task_bids': '{}', "path": '[]', 'slots': '[]', "dist": 0.0, "cost": 0.0}
							self.comm_alive_checker.sql_update_task(task['id'], task_dict)
						else:
							self.comm_alive_checker.sql_delete_from_table('global_task_list', 'id', task['id'])

	def progress_tracker(self):

		"""
		This thread tracks the progress of an executing task.

		"""

		# Loop
		print("AGV-agent:	Progress tracker thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=0.1): 
				print("AGV-agent:	Progress tracker thread killed")
				break

			# Get robot
			robot = self.comm_progress_tracker.sql_select_robot(self.params['id'])
			if robot is None or len(robot) == 0: continue

			# Get task executing
			task_executing = self.comm_progress_tracker.sql_select_executing_task(robot[0]['id'])
			if task_executing is None or len(task_executing) == 0: continue

			# Calculate progress
			progress = round((self.task_executing_dist_done / self.task_executing_total_dist) * 100 if not self.task_executing_total_dist == 0.0 else 0.0)
			
			# Update task
			task_dict = {'progress': progress}
			self.comm_progress_tracker.sql_update_task(task_executing[0]['id'], task_dict)	

	def performance_monitor(self):

		"""
		This thread computes the performance of the allocation by computing the costs of the local task list.

		"""

		# Loop
		print("AGV-agent:	Performance monitor thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=0.1): 
				print("AGV-agent:	Performance monitor thread killed")
				break

			# Get robot
			robot = self.comm_performance_monitor.sql_select_robot(self.params['id'])
			if robot is None or len(robot) == 0: continue

			# Get task executing
			task_executing = self.comm_performance_monitor.sql_select_executing_task(self.id)
			if task_executing is None: continue

			# Get tasks in local task list
			local_task_list = self.comm_performance_monitor.sql_select_local_task_list(self.id)
			if local_task_list is None: continue

			# Get current path
			self.current_path = [] if len(task_executing) == 0 else ast.literal_eval(task_executing[0]['path'])

			# Get total path
			total_path = []
			for task in local_task_list:
				total_path.extend(ast.literal_eval(task['path']))
			self.total_path = total_path

			# Calculate traveled cost
			self.traveled_cost = (datetime.now() - self.start_time).total_seconds()

			# Calculate dist and cost to next node
			dist_to_next = self.dist_euclidean((robot[0]['x_loc'], robot[0]['y_loc']), self.comm_performance_monitor.sql_select_graph().nodes[robot[0]['next_node']].pos)
			cost_to_next = dist_to_next / robot[0]['speed']		

			# Calculate dists and costs
			self.task_executing_dist = 0.0 if len(task_executing) == 0 else dist_to_next + task_executing[0]['dist'] 
			self.task_executing_cost = 0.0 if len(task_executing) == 0 else cost_to_next + task_executing[0]['cost']
			self.local_task_list_dist = sum([task['dist'] for task in local_task_list])
			self.local_task_list_cost = sum([task['cost'] for task in local_task_list])

			# If no routing
			start_node = self.node if len(task_executing) == 0 else task_executing[0]['node'] 
			self.local_task_list_cost = self.get_local_task_list_cost(start_node, local_task_list) if self.local_task_list_cost == 0 else self.local_task_list_cost
			self.task_executing_cost = cost_to_next + self.dist_astar(robot[0]['next_node'], start_node)[1] if self.task_executing_cost == 0 else self.task_executing_cost

			print("Update:")
			print("\tNext node: " + str(robot[0]['next_node']))
			print("\tTask executing: " + str([task['node'] for task in task_executing]))
			print("\tLocal tasks: " + str([task['node'] for task in local_task_list]))
			print("\tCurrent path: " + str(self.current_path))
			print("\tTotal path: " + str(self.total_path))
			print("\tTraveled cost: " + str(self.traveled_cost))
			print("\tcost_to_next: " + str(cost_to_next))
			print("\tTask executing cost: " + str(self.task_executing_cost))
			print("\tLocal task list cost: " + str(self.local_task_list_cost))

	def get_local_task_list_cost(self, start_node, local_task_list):
		total_dist = 0.0
		first_node = start_node
		for next_node in local_task_list:
			_, dist = self.dist_astar(first_node, next_node['node'])
			total_dist += dist
			first_node = next_node['node']
		return total_dist

	def dist_astar(self, a, b):
		return get_shortest_path(self.graph, a, b)

	@staticmethod
	def dist_euclidean(a, b):
		return math.sqrt(math.pow(b[0] - a[0], 2) + math.pow(b[1] - a[1], 2))