import time
import os 
import yaml
import threading

from robotino_core.Comm import Comm
from robotino_core.solvers.astar_solver import *
from robotino_core.solvers.resource_management_solver import *

class AGV_RM_agent:

	"""
			A class containing the intelligence of the Resource Management agent
	"""

	def __init__(self, params_file):

		# Read params
		this_dir = os.path.dirname(os.path.dirname(__file__))
		data_path = os.path.join(this_dir, "params", params_file)
		with open(data_path, 'r') as file:
			self.params = yaml.load(file, Loader=yaml.FullLoader)

		# Init database communication for all threads
		self.comm_resource_optimizer = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_resource_thresholder = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		
		# Exit event
		self.exit_event = threading.Event()

		# Processes
		self.threads = []
		# self.threads.append(threading.Thread(target=self.resource_optimizer))
		self.threads.append(threading.Thread(target=self.resource_thresholder))

	def signal_handler(self, _, __):
		print("\n\nShutdown RM-agent")
		self.exit_event.set()

	def resource_optimizer(self):

		"""
		This thread determines the most optimal resource refill station.

		"""

		# Loop
		print("\nRM-agent:	Resource optimizer thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=self.params['resource_optimizer_rate']): 
				print("\nRM-agent:	Resource optimizer thread killed")
				break

			# Get robot
			robot = self.comm_resource_optimizer.sql_select_robot(self.params['id'])
			if robot is None or len(robot) == 0: continue

			# Get task executing
			task_executing = self.comm_resource_optimizer.sql_select_executing_task(robot[0]['id'])
			if task_executing is None or len(task_executing) == 0: continue

			# Get local task list
			local_task_list = self.comm_resource_optimizer.sql_select_local_task_list(self.params['id'])
			if local_task_list is None: continue

			# Remove charging station
			adapted_local_task_list = [task for task in local_task_list if not task['message'] == 'charging'] 
			charging_station = [task for task in local_task_list if task['message'] == 'charging']

			# Compute end resource level
			task_executing_charge_loss = self.resource_consumption(robot[0]['task_executing_cost'])
			local_task_list_charge_loss = self.resource_consumption(robot[0]['local_task_list_cost'])
			total_charge_loss = task_executing_charge_loss + local_task_list_charge_loss
			end_level = robot[0]['battery_status'] - total_charge_loss

			# Solve for optimal charging station
			if end_level > self.params['battery_threshold']: continue

			# Compute optimal insertion point, charging station, and charging percent
			charging_station, insertion_index, charging_percent = solve_brute_force(adapted_local_task_list, task_executing[0], robot[0]['battery_status'])

			# If no need to charge:
			if insertion_index is None: continue

			# Add or update charging station TODO aad charging percent to database
			if len(charging_station) == 0:
				task_dict = {"node": charging_station, "priority": insertion_index, "robot": self.params['id'], "estimated_start_time": '-', "estimated_end_time": "-", "estimated_duration": "-", "real_start_time": "-", "real_end_time": "-", "real_duration": "-", "progress": 0.0, "message": 'charging', "status": 'assigned', "approach": '-', 'task_bids': '-', 'path': '[]', 'slots': '[]', 'dist': 0.0, 'cost': 0.0}
				self.comm_resource_optimizer.sql_add_to_table('global_task_list', task_dict)
			else:
				task_dict = {"node": charging_station[0]['node'], "priority": insertion_index, "robot": self.params['id'], "message": 'charging', "status": 'assigned'}
				self.comm_resource_optimizer.sql_update_task(charging_station[0]['id'], task_dict)

			# Update local_task_list sequence
			for task in adapted_local_task_list:
				if task['priority'] >= insertion_index:
					task_dict = {'priority': task['priority'] + 1}
					self.comm_resource_optimizer.sql_update_task(task['id'], task_dict)

	def resource_thresholder(self):

		"""
		This thread determines the closest charging station when the resource threshold is reached.

		"""

		# Loop
		print("RM-agent:	Resource thresholder thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=self.params['resource_thresholder_rate']): 
				print("RM-agent:	Resource thresholder thread killed")
				break

			# Get graph
			self.graph = self.comm_resource_thresholder.sql_select_graph()
			if self.graph is None: return None

			# Get task executing
			task_executing = self.comm_resource_thresholder.sql_select_executing_task(self.params['id'])
			if task_executing is None or len(task_executing) == 0: continue

			# Get robot
			robot = self.comm_resource_thresholder.sql_select_robot(self.params['id'])
			if robot is None or len(robot) == 0: continue

			# Check charging status
			if robot[0]['status'] == 'BUSY' and robot[0]['battery_status'] < self.params['battery_threshold']:

				# Get start node of robot
				start_node = task_executing[0]['node']

				# Update robot
				robot_dict = {"status": 'EMPTY'}
				self.comm_resource_thresholder.sql_update_robot(self.params['id'], robot_dict)

				# Create charging task
				closest_charging_station = min(self.params['depot_locations'], key=lambda pos: self.dist_astar(pos, start_node))
				task_dict = {"node": closest_charging_station, "priority": 0, "robot": self.params['id'], "estimated_start_time": '-', "estimated_end_time": "-", "estimated_duration": "-", "real_start_time": "-", "real_end_time": "-", "real_duration": "-", "progress": 0.0, "message": 'charging', "status": 'assigned', "approach": '-', 'task_bids': '-', 'path': '[]', 'slots': '[]', 'dist': 0.0, 'cost': 0.0}
				self.comm_resource_thresholder.sql_add_to_table('global_task_list', task_dict)

	def dist_astar(self, a, b):
		return get_shortest_path(self.graph, a, b)

	def resource_consumption(self, traveltime):
		return traveltime * self.params['resource_consumption_factor']
	