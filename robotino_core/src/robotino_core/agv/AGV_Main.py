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
		A class containing the intelligence of the agv agent
	"""

	def __init__(self, params_file):

		# Read params
		this_dir = os.path.dirname(os.path.dirname(__file__))
		data_path = os.path.join(this_dir, "params", params_file)
		with open(data_path, 'r') as file:
			self.params = yaml.load(file, Loader=yaml.FullLoader)
		
		# Communication layers
		self.comm_main = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_main.sql_open()
		self.comm_odom_callback = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_odom_callback.sql_open()
		self.comm_alive_checker = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_alive_checker.sql_open()
		self.comm_homing = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_homing.sql_open()
		self.comm_progress_tracker = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_progress_tracker.sql_open()
		self.comm_performance_monitor = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_performance_monitor.sql_open()
		self.comm_threshold_charging = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_threshold_charging.sql_open()

		# Action layer		
		self.action = AGV_Action(self)

		# Depot station
		self.depot = self.params['depot'] #random.choice(list(self.comm_main.sql_get_graph().nodes.keys()))

		# Current attributes - status at the moment
		self.id = self.params['id']
		self.x_loc = self.comm_main.sql_get_graph().nodes[self.depot].pos[0]
		self.y_loc = self.comm_main.sql_get_graph().nodes[self.depot].pos[1]
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
		self.comm_main.sql_drop_table('robot' + str(self.id))
		self.comm_main.sql_create_robot_table('robot' + str(self.id))

		# Set graph
		self.graph = self.comm_main.sql_get_graph()

		# Exit event
		self.exit_event = threading.Event()

		# Processes
		self.threads = []
		self.threads.append(threading.Thread(target=self.main))
		self.threads.append(threading.Thread(target=self.odom_callback))
		self.threads.append(threading.Thread(target=self.alive_checker))
		self.threads.append(threading.Thread(target=self.homing))
		self.threads.append(threading.Thread(target=self.progress_tracker))
		self.threads.append(threading.Thread(target=self.performance_monitor))
		self.threads.append(threading.Thread(target=self.threshold_charging))

	def main(self):

		# Loop
		print("\nAGV-agent:	Main thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=0.1): 
				print("\nAGV-agent:	Main thread killed")
				break

			# Wait for a task on the local task list and pick first (lowest priority)
			local_task_list = self.comm_main.sql_get_local_task_list(self.id)
			if local_task_list is None or len(local_task_list) == 0: continue
			task_executing = local_task_list[0]

			# Start task
			print("\nAGV-agent:	Start executing task " + str(task_executing['id']))
			if self.status != 'EMPTY': self.status = 'BUSY'

			# Remove from local task list and add task to executing list
			task_executing_start_time = datetime.now()
			self.task_executing_total_dist = get_shortest_path(self.comm_main.sql_get_graph(), self.node, task_executing['node'])[1]
			task_dict = {'priority': 0, 'status': 'executing', 'estimated_start_time': task_executing_start_time.strftime('%H:%M:%S'), 'real_start_time': task_executing_start_time.strftime('%H:%M:%S')}
			self.comm_main.sql_update_task(task_executing['id'], task_dict)

			#exit()

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
				self.comm_main.sql_update_task(task_executing['id'], task_dict)

				# Reset executing task
				print("AGV-agent:	Finished task " + str(task_executing['id']))
					
			# If task not reached
			else:

				# Make task unnasigned if not able to reach
				task_dict = {'status': 'unassigned', 'message': '-', 'priority': 0}
				self.comm_main.sql_update_task(task_executing['id'], task_dict)

				# Reset executing task
				print("AGV-agent:	Failed task " + str(task_executing['id']))
					
			# Set status to IDLE when task is done or when done charging
			if self.status != 'EMPTY': self.status = 'IDLE'

	def execute_task(self, task):

		# Move from node to node until end node reached
		self.task_executing_dist_done = 0
		while not self.node == task['node']:

			# Get executing task
			executing_task = self.comm_main.sql_get_executing_task(self.id)
			if executing_task is None or len(executing_task) == 0: return False

			# Get planned path
			task_executing_path = ast.literal_eval(executing_task[0]['path'])
			task_executing_slots = [[datetime.now(), timedelta()]] # TODO add real slot

			# Plan path if no path available
			if len(task_executing_path) == 0:
				print("AGV-agent:	Needs to plan astar")
				task_executing_path = get_shortest_path(self.comm_main.sql_get_graph(), self.node, task['node'])[0][1:]
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
			node_position = self.comm_main.sql_get_graph().nodes[self.next_node].pos	
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

	def odom_callback(self):

		# Loop
		print("AGV-agent:	Odom thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=0.1): 
				time.sleep(0.5)
				print("AGV-agent:	Odom thread killed")
				break

			# Make robot dict
			robot_dict = {"id": self.id, "ip": self.params['ip'], "port": self.params['port'], "x_loc": self.x_loc, "y_loc": self.y_loc, "theta": self.theta, "speed": self.speed, "node": self.node,
				"status": self.status, "battery_status": self.battery_status, "sql_queries": self.sql_queries, "traveled_cost": self.traveled_cost, "traveled_dist": self.traveled_dist, "charged_time": self.charged_time,
				"congestions": self.congestions, "next_node": self.next_node, "current_path": str(self.current_path), "total_path": str(self.total_path), 
				"task_executing_dist": self.task_executing_dist, "task_executing_cost": self.task_executing_cost, "local_task_list_dist": self.local_task_list_dist, "local_task_list_cost": self.local_task_list_cost, "time_now": datetime.now().strftime('%H:%M:%S')}

			# Update robot
			self.comm_odom_callback.sql_add_to_table('global_robot_list', robot_dict)
			self.comm_odom_callback.sql_update_robot(self.id, robot_dict)

			# Make robot dict
			robot_dict = {"x_loc": self.x_loc, "y_loc": self.y_loc, "theta": self.theta, "speed": self.speed, "node": self.node,
				"status": self.status, "battery_status": self.battery_status, "sql_queries": self.sql_queries, "traveled_cost": self.traveled_cost, "traveled_dist": self.traveled_dist, "charged_time": self.charged_time,
				"congestions": self.congestions, "next_node": self.next_node, "current_path": str(self.current_path), "total_path": str(self.total_path), 
				"task_executing_dist": self.task_executing_dist, "task_executing_cost": self.task_executing_cost, "local_task_list_dist": self.local_task_list_dist, "local_task_list_cost": self.local_task_list_cost, "time_now": datetime.now().strftime('%H:%M:%S')}

			# Log data
			self.comm_odom_callback.sql_add_to_table('robot' + str(self.id), robot_dict)

			# Update graph
			self.graph = self.comm_odom_callback.sql_get_graph()

	def alive_checker(self):

		# Loop
		print("AGV-agent:	Alive checker thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=0.1): 
				time.sleep(0.6)
				print("AGV-agent:	Alive checker thread killed")
				break

			# Get all robots
			robots = self.comm_alive_checker.sql_select_everything_from_table('global_robot_list')
			if robots is None or len(robots) == 0: continue
				
			# Loop over all robots
			for robot in robots:

				# Skip this robot
				if robot['id'] == self.id: continue

				# Compute time difference
				robot_time = datetime.strptime(robot['time_now'], '%H:%M:%S').time()
				now = (datetime.now() - timedelta(seconds = 5)).time()

				# Delete if time differs too much
				if robot_time < now:

					# Print robot falldown detection
					print("AGV " + str(self.id) + " :   	 	Detected fallout of robot " + str(robot['id']))

					# Delete robot from database
					self.comm_alive_checker.sql_delete_from_table('global_robot_list', 'id', robot['id'])
						
					# Make all tasks unassigned
					local_task_list = self.comm_alive_checker.sql_get_local_task_list(robot['id'])
					task_executing = self.comm_alive_checker.sql_get_executing_task(robot['id'])
					for task in local_task_list + task_executing: 
						if not task['message'] in ['homing', 'charging']:
							task_dict = {'priority': 0, 'robot':-1, 'estimated_start_time': '-', 'estimated_end_time': '-', 'estimated_duration': '-', 'real_start_time': '-', 'real_end_time': '-', 'real_duration': '-', 'progress': 0.0, 'message': '-', 'status': 'unassigned', 'approach': task['approach'], 'task_bids': '-', 'path': '[]', 'slots': '[]'}
							self.comm_alive_checker.sql_update_task(task['id'], task_dict)
						else:
							self.comm_alive_checker.sql_delete_from_table('global_task_list', 'id', task['id'])

	def homing(self):

		# Loop
		print("AGV-agent:	Homing thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=self.params['ro_rate']): 
				time.sleep(0.7)
				print("AGV-agent:	Homing thread killed")
				break

			# Get task executing
			task_executing = self.comm_homing.sql_get_executing_task(self.id)
			if task_executing is None or len(task_executing) == 0: continue
			task_executing = task_executing[0]

			# Get tasks in local task list
			local_task_list = self.comm_homing.sql_get_local_task_list(self.id)
			if local_task_list is None: continue

			# Add homing task
			if not self.node == self.depot and len([task for task in local_task_list if task['message'] == 'homing']) == 0 and not task_executing['message'] == 'homing':

				# Add task to task list
				task_dict = {"node": self.depot, "priority": 1, "robot": self.id, "estimated_start_time": '-', "estimated_end_time": "-", "estimated_duration": "-", "real_start_time": "-", "real_end_time": "-", "real_duration": "-", "progress": 0.0, "message": 'homing', "status": 'assigned', "approach": '-', 'task_bids': '-', 'path': '[]', 'slots': '[]', 'dist': 0.0, 'cost': 0.0}
				self.comm_homing.sql_add_to_table('global_task_list', task_dict)

	def progress_tracker(self):

		# Loop
		print("AGV-agent:	Progress tracker thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=0.1): 
				time.sleep(0.8)
				print("AGV-agent:	Progress tracker thread killed")
				break

			# Get robot
			robot = self.comm_progress_tracker.sql_get_robot(self.params['id'])
			if robot is None or len(robot) == 0: continue
			robot = robot[0]

			# Get task executing
			task_executing = self.comm_progress_tracker.sql_get_executing_task(robot['id'])
			if task_executing is None or len(task_executing) == 0: continue
			task_executing = task_executing[0]

			# Calculate progress
			progress = round((self.task_executing_dist_done / self.task_executing_total_dist) * 100 if not self.task_executing_total_dist == 0.0 else 0.0)
			task_dict = {'progress': progress}
			self.comm_progress_tracker.sql_update_task(task_executing['id'], task_dict)	

	def performance_monitor(self):

		# Loop
		print("AGV-agent:	Performance monitor thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=0.1): 
				time.sleep(0.9)
				print("AGV-agent:	Performance monitor thread killed")
				break

			# Calculate traveled cost
			self.traveled_cost = (datetime.now() - self.start_time).total_seconds()

			# Calculate sql queries
			self.sql_queries = self.comm_performance_monitor.sql_queries + self.comm_performance_monitor.sql_queries

			# Get robot
			robot = self.comm_performance_monitor.sql_get_robot(self.params['id'])
			if robot is None or len(robot) == 0: continue
			robot = robot[0]

			# Get task executing
			task_executing = self.comm_performance_monitor.sql_get_executing_task(self.id)
			if task_executing is None: continue

			# Get tasks in local task list
			local_task_list = self.comm_performance_monitor.sql_get_local_task_list(self.id)
			if local_task_list is None: continue

			# Calculate dist and cost to next node
			dist_to_next = self.dist_euclidean((robot['x_loc'], robot['y_loc']), self.comm_performance_monitor.sql_get_graph().nodes[robot['next_node']].pos)
			cost_to_next = dist_to_next / robot['speed']		

			# Get dists and costs
			self.task_executing_dist = 0.0 if len(task_executing) == 0 else dist_to_next + task_executing[0]['dist'] 
			self.task_executing_cost = 0.0 if len(task_executing) == 0 else cost_to_next + task_executing[0]['cost']
			self.local_task_list_dist = sum([task['dist'] for task in local_task_list])
			self.local_task_list_cost = sum([task['cost'] for task in local_task_list])

			# Get current path
			self.current_path = [] if len(task_executing) == 0 else ast.literal_eval(task_executing[0]['path'])

			# Get total path
			total_path = []
			for task in local_task_list:
				total_path.extend(ast.literal_eval(task['path']))
			self.total_path = total_path

	def threshold_charging(self):

		# Loop
		print("AGV-agent:	Threshold charging thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=0.1): 
				time.sleep(1.0)
				print("AGV-agent:	Threshold charging thread killed")
				break

			# Get task executing
			task_executing = self.comm_threshold_charging.sql_get_executing_task(self.id)
			if task_executing is None or len(task_executing) == 0: continue
			task_executing = task_executing[0]

			# Check charging status
			if self.status == 'BUSY':
				if self.battery_status < self.params['battery_threshold']:

					# Update status to empty
					self.status = 'EMPTY'

					# Get start node of robot
					start_node = task_executing['node']

					# Create charging task
					closest_charging_station = self.search_closest_charging_station(start_node)
					task_dict = {"node": closest_charging_station, "priority": 0, "robot": self.id, "estimated_start_time": '-', "estimated_end_time": "-", "estimated_duration": "-", "real_start_time": "-", "real_end_time": "-", "real_duration": "-", "progress": 0.0, "message": 'charging', "status": 'assigned', "approach": '-', 'task_bids': '-', 'path': '[]', 'slots': '[]', 'dist': 0.0, 'cost': 0.0}
					self.comm_threshold_charging.sql_add_to_table('global_task_list', task_dict)

	def search_closest_charging_station(self, loc):
		node = min(self.params['depot_locations'], key=lambda pos: self.dist_astar(pos, loc))
		return node

	def dist_astar(self, a, b):
		return get_shortest_path(self.graph, a, b)

	def search_closest_node(self, loc):
		node = min(self.graph.nodes.values(), key=lambda node: self.dist_euclidean(node.pos, loc))
		return node.name

	@staticmethod
	def dist_euclidean(a, b):
		return math.sqrt(math.pow(b[0] - a[0], 2) + math.pow(b[1] - a[1], 2))
