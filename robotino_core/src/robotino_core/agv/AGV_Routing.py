import time
from datetime import date, datetime, timedelta

from robotino_core.solvers.feasibility_ant_solver import *
from robotino_core.Comm import Comm

import numpy as np

class Routing:
	"""
			A class containing the intelligence of the Routing agent
	"""

	def __init__(self, agv):

		# AGV
		self.agv = agv

		# Last recorded reservations
		self.last_reservations = []

	def main(self):

		# Open database connection
		print("   Routing agent:             Started")
		comm = Comm(self.agv.ip, self.agv.port, self.agv.host, self.agv.user, self.agv.password, self.agv.database)
		comm.sql_open()
		
		# Loop
		while True:

			# Timeout
			time.sleep(self.agv.params['dmas_rate'])

			# Plan if there are tasks
			if not self.agv.task_executing['id'] == -1:

				# Plan path and slots towards executing task
				self.plan_executing_task(comm)

				# Plan path and slots towards local tasks
				self.plan_local_tasks(comm)

			# Close thread at close event 
			if self.agv.exit_event.is_set():
				break

	def plan_executing_task(self, comm):

		# Init
		start_node = self.agv.current_node
		estimated_start_time = datetime.now() + timedelta(seconds=self.agv.calculate_euclidean_distance((self.agv.x_loc, self.agv.y_loc), self.agv.graph.nodes[self.agv.current_node].pos) / self.agv.params['robot_speed'])

		# Nodes to visit
		node_to_visit = [self.agv.task_executing['node']]

		# If end node not reached
		if not start_node == self.agv.task_executing['node']:

			# Do dmas
			best_path, best_slots = self.dmas(start_node, node_to_visit, estimated_start_time, comm)

			# Estimated end time and duration
			estimated_end_time = best_slots[-1][0] + best_slots[-1][1]
			estimated_duration = estimated_end_time - self.agv.start_time

			# Reserve slots
			self.intent(best_path, best_slots, comm)

			# Set paths towards all tasks
			self.agv.reserved_paths[self.agv.task_executing['id']] = best_path
			self.agv.reserved_slots[self.agv.task_executing['id']] = best_slots

			# Set paths towards all tasks
			self.agv.estimated_end_time = estimated_end_time

		else:

			# Estimated end time and duration
			time_to_do = timedelta(seconds=self.agv.calculate_euclidean_distance((self.agv.x_loc, self.agv.y_loc), self.agv.graph.nodes[self.agv.current_node].pos) / self.agv.params['robot_speed'])
			estimated_end_time = datetime.now() + time_to_do
			estimated_duration = estimated_end_time - self.agv.start_time

			# Set paths towards all tasks
			self.agv.reserved_paths[self.agv.task_executing['id']] = []
			self.agv.reserved_slots[self.agv.task_executing['id']] = []

		# Update executing task
		task_dict = {'estimated_end_time': estimated_end_time.strftime('%H:%M:%S'), 'estimated_duration': str(estimated_duration)}
		comm.sql_update_task(self.agv.task_executing['id'], task_dict)

	def plan_local_tasks(self, comm):

		# Init
		start_node = self.agv.task_executing['node']
		estimated_start_time = self.agv.estimated_end_time

		# Get tasks in local task list
		local_task_list = comm.sql_get_local_task_list(self.agv.id)

		# Nodes to visit
		nodes_to_visit = local_task_list if not local_task_list is None else []

		# Update paths for each task in local task list
		self.agv.total_path = []
		updated_tasks = []
		for task in nodes_to_visit:

			# Do dmas
			best_path, best_slots = self.dmas(start_node, [task['node']], estimated_start_time, comm)

			# Estimated end time and duration
			estimated_end_time = best_slots[-1][0] + best_slots[-1][1]
			estimated_duration = estimated_end_time - estimated_start_time

			# Reserve slots
			self.intent(best_path, best_slots, comm)

			# Set paths towards all tasks
			self.agv.reserved_paths[task['id']] = best_path
			self.agv.reserved_slots[task['id']] = best_slots

			# Set paths towards all tasks
			self.agv.total_path.extend(best_path)

			# Update task
			updated_tasks.append((self.agv.id, task['status'], task['message'], task['priority'], estimated_start_time.strftime('%H:%M:%S'), estimated_end_time.strftime('%H:%M:%S'), str(estimated_duration), '-', '-', '-', 0, task['id']))
						
			# Estimated start time for next task
			estimated_start_time = estimated_end_time

			# Start node for next task is end node of previous task
			start_node = task['node']

		# Update tasks
		comm.sql_update_tasks(updated_tasks)

	def dmas(self, start_node, nodes_to_visit, start_time, comm):

		# Assertions
		assert isinstance(start_node, str)
		assert isinstance(nodes_to_visit, list)
		assert isinstance(start_time, datetime)
		assert len(nodes_to_visit) > 0

		# Feasibility ants
		local_feasible_paths = self.think(start_node, nodes_to_visit)

		# If feasible paths exist
		if local_feasible_paths:

			# Exploration ants
			explored_paths, fitness_values, slots = self.explore(local_feasible_paths, start_time, comm)

			# Best route selection
			best_path = explored_paths[int(np.argmin(fitness_values))]
			best_slots = slots[int(np.argmin(fitness_values))]

			return best_path[1:], best_slots

		else:
			print("No feasible paths found")
			return None, None

	def think(self, start_node, nodes_to_visit):

		# Assertions
		assert isinstance(start_node, str)
		assert isinstance(nodes_to_visit, list)
		assert len(nodes_to_visit) > 0

		# Find feasible paths
		_, local_best_solutions = feasibility_ant_solve(self.agv.graph, start_node, nodes_to_visit)
		return local_best_solutions

	def explore(self, paths, start_time, comm):

		# Assertions
		assert isinstance(paths, list)
		assert isinstance(start_time, datetime)

		# Init
		fitness_values = []
		total_travel_costs = []
		total_delays = []
		all_slots = []

		# Explore paths
		for path in paths:

			# Init
			timestamp = start_time
			total_delay = timedelta()
			total_travel_time = timedelta()
			slots = []

			# Calculate slot of nodes in between
			for i in range(0, len(path) - 1):

				# Calculate traveltime to drive to node i+1
				travel_time = timedelta(seconds=self.agv.graph.edges[path[i], path[i + 1]].length / self.agv.params['robot_speed'])
				wanted_slot = (timestamp, travel_time)

				# Check available slots for node i+1
				slot, delay = self.check_slot(path[i+1], wanted_slot, comm)

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

		return paths, fitness_values, all_slots

	def intent(self, path, slots, comm):
		for i in range(len(path)):
			self.reserve_slot(path[i], slots[i], comm)
			
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
					task_dict = {"node": self.agv.depot, "priority": 1, "robot": self.agv.id, "message": 'homing', "status": 'assigned', 'estimated_start_time': datetime.now().strftime('%H:%M:%S')}
					comm.sql_add_to_table('global_task_list', task_dict)

	def check_slot(self, node, slot, comm):

		# Assertions
		assert isinstance(node, str)
		assert isinstance(slot[0], datetime)
		assert isinstance(slot[1], timedelta)

		# Check all available slots of other robots
		available_slots = self.get_available_slots(node, slot, comm)

		# Take first available slot and adapt the end time to the required end time
		first_available_slot = available_slots[0]
		first_available_slot = (first_available_slot[0], slot[1])

		# Compute delay
		delay = first_available_slot[0] - slot[0]
		return first_available_slot, delay

	def get_available_slots(self, node, slot, comm):

		# Assertions
		assert isinstance(node, str)
		assert isinstance(slot[0], datetime)
		assert isinstance(slot[1], timedelta)

		# Get slot information
		reservation_time = slot[0]
		duration = slot[1]

		# Get all reservations
		time.sleep(0.1)
		reservations = comm.sql_select_from_table('environmental_agents', 'node', node)
		#if reservations is None:
			#reservations = self.last_reservations
		#else: 
			#self.last_reservations = reservations

		# Get all reservations from the requested reservation_time for all other robots
		slots = []
		for res in reservations:
			start_time = datetime.strptime(res['start_time'], '%Y-%m-%d %H:%M:%S')
			end_time = datetime.strptime(res['end_time'], '%Y-%m-%d %H:%M:%S')
			if end_time > reservation_time and not res['robot'] == self.agv.id:
				slots.append((start_time, end_time))

		# Get free slots
		free_slots = []
		if slots:
			# Free slot from requested reservation time till start time of eariest reservation
			if slots[0][0] - reservation_time >= duration:
				free_slots.append((reservation_time, slots[0][0]))
			# Free intermediate slots
			for start, end in ((slots[i][1], slots[i + 1][0]) for i in range(len(slots) - 1)):
				if end - start >= duration:
					free_slots.append((start, end - start))
			# Free slot from last reservation time till infinity
			free_slots.append((slots[-1][1], float('inf')))
		else:
			free_slots.append((reservation_time, float('inf')))
		return free_slots

	def reserve_slot(self, node, slot, comm):

		# Assertions
		assert isinstance(node, str)
		assert isinstance(slot[0], datetime)
		assert isinstance(slot[1], timedelta)

		# Get slot
		reservation_start_time = slot[0]
		reservation_end_time = reservation_start_time + slot[1]

		# Get all reservations
		reservations = comm.sql_select_from_table('environmental_agents', 'node', node)

		valid = True
		for res in reservations:
			start_time = datetime.strptime(res['start_time'], '%Y-%m-%d %H:%M:%S')
			end_time = datetime.strptime(res['end_time'], '%Y-%m-%d %H:%M:%S')
			if not res['robot'] == self.agv.id:
				if reservation_start_time <= start_time < reservation_end_time or reservation_start_time < end_time <= reservation_end_time:
					valid = False
					break
		if valid:
			reservation_dict = {'node': node, 'robot': self.agv.id, 'start_time': reservation_start_time.strftime('%Y-%m-%d %H:%M:%S'), 'end_time': reservation_end_time.strftime('%Y-%m-%d %H:%M:%S'), 'pheromone': 100.0}
			id = comm.sql_add_to_table('environmental_agents', reservation_dict)
			# print("Slot (" + str(id) + ', '+ str(reservation_start_time) + ', ' + str(reservation_end_time) + ") reserved for agv " + str(self.agv.id))
			return id
		else:
			print("Could not add slot (" + str(reservation_start_time) + ', ' + str(reservation_end_time) + ") of agv " + str(self.agv.id))
			return None
