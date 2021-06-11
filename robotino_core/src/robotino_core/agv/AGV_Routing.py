import time
from datetime import datetime, timedelta

from robotino_core.solvers.feasibility_ant_solver import *
from robotino_core.Comm import Comm

import numpy as np

class Routing:
	"""
			A class containing the intelligence of the Routing agent
	"""

	def __init__(self, agv):

		# agv
		self.agv = agv

	def main(self):

		# Open database connection
		print("   Routing agent:             Started")
		comm = Comm(self.agv.ip, self.agv.port, self.agv.host, self.agv.user, self.agv.password, self.agv.database)
		comm.sql_open()
		
		# Loop
		while True:

			# Timeout
			time.sleep(self.agv.params['dmas_update_rate'])

			# Get tasks in local task list
			local_task_list = comm.sql_get_local_task_list(self.agv.id)

			# Get start node of robot
			start_node = self.agv.task_executing['node'] if not self.agv.task_executing['id'] == -1 else self.agv.node

			# Get start time of robot
			start_time = datetime.now() #if self.agv.task_executing['id'] == -1 and len(self.agv.slots) == 0 else self.agv.slots[-1][0] + self.agv.slots[-1][1]

			# Update paths for each task in local task list
			total_path = []
			for task in local_task_list:

				# Do dmas
				best_path, best_slots, _ = self.dmas(start_node, [task['node']], start_time)

				if best_path:

					# Set paths towards all tasks
					self.agv.reserved_paths[task['id']] = best_path
					self.agv.reserved_slots[task['id']] = best_slots

					# Set total planned path
					total_path.extend(best_path)

					# Start node for next task is end node of previous task
					start_node = task['node']

					# Starting time for next task
					start_time = best_slots[-1][0] + best_slots[-1][1]

					# Reserve slots
					self.intent(self.agv.id, best_path, best_slots)

			# Update total path
			self.agv.total_path = total_path

			# Close thread at close event 
			if self.agv.exit_event.is_set():
				break

	def dmas(self, start_node, nodes_to_visit, start_time):

		# Feasibility ants
		_, local_feasible_paths = self.think(start_node, nodes_to_visit)

		# If feasible paths exist
		if local_feasible_paths:

			# Exploration ants
			explored_paths, fitness_values, _, total_delays, slots = self.explore(self.agv.id, local_feasible_paths, start_time)

			# Best route selection
			best_path = explored_paths[int(np.argmin(fitness_values))]
			best_slots = slots[int(np.argmin(fitness_values))]
			best_delay = total_delays[int(np.argmin(fitness_values))]

			return best_path[1:], best_slots, best_delay

		else:
			print("No feasible paths found")
			return None, None, None

	def think(self, start_node, nodes_to_visit):
		global_best_solution, local_best_solutions = feasibility_ant_solve(self.agv.graph, start_node, nodes_to_visit)
		return global_best_solution, local_best_solutions

	def explore(self, agv_id, paths, start_time):

		# Init
		fitness_values = []
		total_travel_costs = []
		total_delays = []
		all_slots = []

		# Explore paths
		for path in paths:

			# Init
			timestamp = start_time
			total_delay = timedelta(0, 0)
			total_travel_time = timedelta(0, 0)
			slots = []

			# Calculate slot of nodes in between
			for i in range(0, len(path) - 1):

				# Calculate traveltime to drive to node i+1
				travel_time = timedelta(0, self.agv.graph.edges[path[i], path[i + 1]].length / self.agv.params['robot_speed'])
				wanted_slot = (timestamp, travel_time)

				# Check available slots for node i+1
				slot, delay = self.agv.graph.nodes[path[i+1]].environmental_agent.check_slot(wanted_slot, agv_id)

				# Append slot and update state
				slots.append(slot)
				total_travel_time += travel_time
				total_delay += delay
				timestamp += travel_time + delay

			# Collect results
			fitness_values.append(timestamp)
			total_travel_costs.append(total_travel_time)
			total_delays.append(total_delay)
			all_slots.append(slots)

		return paths, fitness_values, total_travel_costs, total_delays, all_slots

	def intent(self, agv_id, path, slots):

		for i in range(len(path)):
			# Destination node
			dest = path[i]

			# Wanted slot
			wanted_slot = slots[i]

			# Reserve slot
			self.agv.graph.nodes[dest].environmental_agent.reserve_slot(wanted_slot, agv_id)

	def homing(self):

		# Open database connection
		comm = Comm(self.agv.ip, self.agv.port, self.agv.host, self.agv.user, self.agv.password, self.agv.database)
		comm.sql_open()

		# Loop
		while True:

			# Get tasks in local task list
			local_task_list = comm.sql_get_local_task_list(self.agv.id)

			# If database alive
			if not local_task_list is None:

				# Add homing task if all work is done
				if len(local_task_list) == 0 and self.agv.task_executing['id'] == -1 and not self.agv.node == self.agv.depot:

					# Add task to task list
					task_dict = {"node": self.agv.depot, "priority": 1, "robot": self.agv.id, "message": 'homing', "status": 'assigned'}
					comm.sql_add_to_table('global_task_list', task_dict)
