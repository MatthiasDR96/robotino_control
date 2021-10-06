import time
import os 
import yaml

from robotino_core.Comm import Comm
from robotino_core.solvers.astar_solver import *

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

		# Init database communication
		self.comm_main = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_main.sql_open()
		self.comm_threshold_charging = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_threshold_charging.sql_open()

	def main(self, _, stop):

		# Loop
		print("RM-agent:	Main thread started")
		while True:

			# Timeout
			time.sleep(self.agv.params['dmas_rate'])

			# Get start node and local task list
			start_task = self.agv.task_executing
			local_task_list = self.comm_main.sql_get_local_task_list(self.agv.id)

			# Compute if there arelocal task lists
			if start_task['id'] == -1 or len(local_task_list) == 0:
				continue

			# Remove charging station
			adapted_local_task_list = [task for task in local_task_list if not task['message'] == 'charging'] 

			# Task executing cost
			task_executing_cost = self.agv.task_executing_cost

			# Local task list cost
			local_task_list_cost = self.agv.local_task_list_cost

			# If path already planned
			if not all(elem in local_task_list_cost.keys() for elem in [task['id'] for task in adapted_local_task_list]):
				continue

			# Compute end resource level
			task_executing_charge_loss = self.resource_consumption(task_executing_cost)
			local_task_list_charge_loss = [self.resource_consumption(cost) for cost in local_task_list_cost.values()]
			total_charge_loss = task_executing_charge_loss + sum(local_task_list_charge_loss)
			end_level = self.agv.battery_status - total_charge_loss

			# Solve for optimal charging station
			if not end_level <= self.agv.params['battery_threshold']:
				continue

			# Compute optimal insertion point, charging station, and charging percent
			charging_station, insertion_index, charging_percent = self.optimize_brute_force(adapted_local_task_list, start_task, self.agv.battery_status)

			# If no need to charge:
			if insertion_index is None:
				continue

			# Add or update charging station
			old_charging_station = [task for task in local_task_list if task['message'] == 'charging']
			if len(old_charging_station) == 0:
				task_dict = {"node": charging_station, "priority": insertion_index, "robot": self.agv.id, "estimated_start_time": '-', "estimated_end_time": "-", "estimated_duration": "-", "real_start_time": "-", "real_end_time": "-", "real_duration": "-", "progress": 0.0, "message": 'charging', "status": 'assigned', "approach": '-', 'task_bids': '-', 'path': '[]', 'slots': '[]', 'dist': 0.0, 'cost': 0.0}
				self.comm_main.sql_add_to_table('global_task_list', task_dict)
			else:
				task_dict = {"node": old_charging_station[0]['node'], "priority": insertion_index, "robot": self.agv.id, "message": 'charging', "status": 'assigned'}
				self.comm_main.sql_update_task(old_charging_station[0]['id'], task_dict)

			# Update local_task_list sequence
			for task in adapted_local_task_list:
				if task['priority'] >= insertion_index:
					task_dict = {'priority': task['priority'] + 1}
					self.comm_main.sql_update_task(task['id'], task_dict)

			# Close thread at close event 
			if stop():
				print("RM-agent:	Main thread killed")
				break

	