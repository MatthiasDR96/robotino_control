from datetime import datetime, timedelta
import time
import numpy as np

from robotino_core.solvers.feasibility_ant_solver import *
from robotino_core.Comm import Comm

class RO_agent:
	"""
			A class containing the intelligence of the Routing agent
	"""

	def __init__(self, agv):

		# AGV
		self.agv = agv

		# Init database communication
		self.comm_main = Comm(self.agv.ip, self.agv.port, self.agv.host, self.agv.user, self.agv.password, self.agv.database)
		self.comm_main.sql_open()
		self.comm_homing = Comm(self.agv.ip, self.agv.port, self.agv.host, self.agv.user, self.agv.password, self.agv.database)
		self.comm_homing.sql_open()

	def main(self, _, stop):
		
		# Loop
		print("RO-agent:	Main thread started")
		while True:

			# Timeout
			time.sleep(self.agv.params['dmas_rate'])

			# Plan if there are tasks
			if not self.agv.task_executing['id'] == -1:

				# Plan path and slots towards executing task
				self.plan_executing_task(self.comm_main)

			# Plan if there are tasks
			if not self.agv.task_executing['id'] == -1:

				# Plan path and slots towards local tasks
				self.plan_local_tasks(self.comm_main)

			# Close thread at close event 
			if stop():
				print("RO-agent:	Main thread killed")
				break

	def homing(self, _, stop):

		# Loop
		print("RO-agent:	Homing thread started")
		while True:

			# Get tasks in local task list
			local_task_list = self.comm_homing.sql_get_local_task_list(self.agv.id)

			# If database alive
			if not local_task_list is None:

				# Add homing task if all work is done
				if len(local_task_list) == 0 and self.agv.task_executing['id'] == -1 and not self.agv.node == self.agv.depot:

					# Add task to task list
					task_dict = {"node": self.agv.depot, "priority": 1, "robot": self.agv.id, "estimated_start_time": '-', "estimated_end_time": "-", "estimated_duration": "-", "real_start_time": "-", "real_end_time": "-", "real_duration": "-", "progress": 0.0, "message": 'homing', "status": 'assigned', "approach": '-', 'task_bids': '-'}
					self.comm_homing.sql_add_to_table('global_task_list', task_dict)

			# Close thread at close event 
			if stop():
				print("RO-agent:	Homing thread killed")
				break


	def plan_executing_task(self, comm):

		# Assertions
		assert isinstance(comm, Comm)

		# Init
		start_node = self.agv.current_node
		start_time = datetime.now() + timedelta(seconds=self.agv.calculate_euclidean_distance((self.agv.x_loc, self.agv.y_loc), self.agv.graph.nodes[self.agv.current_node].pos) / self.agv.params['robot_speed'])

		# If end node not reached
		if not start_node == self.agv.task_executing['node']:

			# Do dmas untill successful reservation
			res = False
			while not res:

				# Do dmas
				best_path, best_slots, _, _ = self.dmas(start_node, self.agv.task_executing['node'], start_time, comm)

				# Reserve slots
				res = self.intent(best_path, best_slots, comm)

				# Set paths towards all tasks
				self.agv.reserved_paths[self.agv.task_executing['id']] = best_path
				self.agv.reserved_slots[self.agv.task_executing['id']] = best_slots

		else:

			# Set paths towards all tasks
			self.agv.reserved_paths[self.agv.task_executing['id']] = []
			self.agv.reserved_slots[self.agv.task_executing['id']] = [(start_time, timedelta())]

		# Get path and slots
		path = self.agv.reserved_paths[self.agv.task_executing['id']]
		slots = self.agv.reserved_slots[self.agv.task_executing['id']]
		
		# Calculate distance
		dist = self.agv.calculate_euclidean_distance((self.agv.x_loc, self.agv.y_loc), self.agv.graph.nodes[self.agv.current_node].pos)
		dist += sum([self.agv.graph.edges[path[i], path[i+1]].dist for i in range(len(path)-1)]) 
					
		# Calculate end time
		end_time = slots[-1][0] + slots[-1][1]

		# Calculate time slot
		start_time = self.agv.task_executing_start_time
		cost = (end_time - start_time).total_seconds()
					
		# Calculate progres
		cost_to_be_done = (end_time - datetime.now()).total_seconds()
		progress = round(((cost - cost_to_be_done) / cost) * 100 if not cost == 0.0 else 0.0)

		# Save attributes
		self.traveled_cost = (datetime.now() - self.agv.start_time).total_seconds()
		self.task_executing_estimated_dist = dist
		self.task_executing_estimated_cost = cost_to_be_done
		self.task_executing_estimated_end_time = end_time

		# Update executing task
		task_dict = {'progress': progress, 'estimated_start_time': self.agv.task_executing_start_time.strftime('%H:%M:%S'), 'estimated_end_time': end_time.strftime('%H:%M:%S'), 'estimated_duration': str(cost)}
		comm.sql_update_task(self.agv.task_executing['id'], task_dict)

	def plan_local_tasks(self, comm):

		# Assertions
		assert isinstance(comm, Comm)

		# Init
		start_node = self.agv.task_executing['node']
		start_time = self.agv.reserved_slots[self.agv.task_executing['id']][-1][0] + self.agv.reserved_slots[self.agv.task_executing['id']][-1][1]

		# Get tasks in local task list
		local_task_list = comm.sql_get_local_task_list(self.agv.id)

		# Do dmas untill successful reservation
		res = False
		while not res:

			# Update local_task_list
			sequence, paths, slots, _, _ = self.agv.tsp_dmas(start_node, start_time, local_task_list, comm)

			# Adapt slots to be correct
			new_slots = copy(slots)
			prev_slot = (slots[0][0][0], timedelta()) if len(slots) > 0 else ()
			for i in range(len(slots)):
				for j in range(len(slots[i])):
					new_slots[i][j] = (prev_slot[0] + prev_slot[1], slots[i][j][1])
					prev_slot = (prev_slot[0] + prev_slot[1], slots[i][j][1])

			# Create total path and slots
			total_path = [item for sublist in paths for item in sublist]
			total_slots = [item for sublist in slots for item in sublist]

			# Set paths towards all tasks
			self.agv.total_path = total_path

			# Update local_task_list sequence
			priority = 1
			for task in sequence:
				task_dict = {'priority': priority}
				comm.sql_update_task(task['id'], task_dict)
				priority += 1

			# Reserve slots
			res = self.intent(total_path, total_slots, comm)

			# Set paths towards all tasks
			end_time = self.task_executing_estimated_end_time
			for i in range(len(sequence)):

				# Save paths and slots
				self.agv.reserved_paths[sequence[i]['id']] = paths[i]
				self.agv.reserved_slots[sequence[i]['id']] = slots[i]

				# Calculate distance
				dist = sum([self.agv.graph.edges[paths[i][j], paths[i][j+1]].dist for j in range(len(paths[i])-1)])

				# Calculate time slot
				start_time = slots[i][0][0]
				end_time = slots[i][-1][0] + slots[i][-1][1]
				cost = (end_time - start_time).total_seconds()

				# Update task
				task_dict = {'estimated_start_time': start_time.strftime('%H:%M:%S'), 'estimated_end_time': end_time.strftime('%H:%M:%S'), 'estimated_duration': str(cost)}
				comm.sql_update_task(task['id'], task_dict)

				# Update local task list dist and cost
				self.agv.local_task_list_estimated_dist[sequence[i]['id']] = dist
				self.agv.local_task_list_estimated_cost[sequence[i]['id']] = cost

			# Update
			self.agv.local_task_list_estimated_end_time = end_time

	def dmas(self, start, dest, start_time, comm):

		# Assertions
		assert isinstance(start, str)
		assert isinstance(dest, str)
		assert isinstance(start_time, datetime)
		assert isinstance(comm, Comm)

		# Feasibility ants
		feasible_paths, _ = get_alternative_paths(self.agv.graph, start, dest, 5)

		# Exploration ants
		fitness_values, slots, dists, costs = self.explore(feasible_paths, start_time, comm)

		# Best route selection
		best_path = feasible_paths[int(np.argmin(fitness_values))]
		best_slots = slots[int(np.argmin(fitness_values))]
		best_dist = dists[int(np.argmin(fitness_values))]
		best_cost = costs[int(np.argmin(fitness_values))]

		return best_path[1:], best_slots, best_dist, best_cost

	def explore(self, paths, start_time, comm):

		# Assertions
		assert isinstance(paths, list)
		assert isinstance(start_time, datetime)
		assert isinstance(comm, Comm)

		# Init
		fitness_values = []
		total_dists = []
		total_costs = []
		all_slots = []

		# Explore paths
		for path in paths:

			# Init
			timestamp = start_time
			total_dist = 0.0
			total_cost = 0.0
			slots = []

			# Calculate slot of nodes in between
			for i in range(0, len(path) - 1):

				# Calculate dist and traveltime to drive to node i+1
				dist = self.agv.graph.edges[path[i], path[i + 1]].dist
				travel_time = timedelta(seconds= dist / self.agv.params['robot_speed'])

				# Get wanted slot
				wanted_slot = (timestamp, travel_time)

				# Check available slots for node i+1
				slot, delay = self.check_slot(path[i+1], wanted_slot, comm)
				
				# Calculate edge cost
				cost = travel_time.total_seconds() + delay.total_seconds()

				# Append slot and update state
				timestamp += travel_time + delay
				total_dist += dist
				total_cost += cost
				slots.append(slot)

			# Collect results
			fitness_values.append(timestamp)
			total_dists.append(total_dist)
			total_costs.append(total_cost)
			all_slots.append(slots)

		return fitness_values, all_slots, total_dists, total_costs

	def intent(self, path, slots, comm):

		# Assertions
		assert isinstance(path, list)
		assert isinstance(slots, list)
		assert isinstance(comm, Comm)

		# Intent
		for i in range(len(path)):
			result = self.reserve_slot(path[i], slots[i], comm)
			if not result:
				return False
		return True
			
	def check_slot(self, node, slot, comm):

		# Assertions
		assert isinstance(node, str)
		assert isinstance(slot[0], datetime)
		assert isinstance(slot[1], timedelta)
		assert isinstance(comm, Comm)

		# Get slot information
		reservation_time = slot[0]
		duration = slot[1]

		# Get all reservations
		reservations = comm.sql_select_reservations('environmental_agents', node,  self.agv.id)

		# Get all reservations from the requested reservation_time for all other robots
		slots = []
		for res in reservations:
			start_time = datetime.strptime(res['start_time'], '%Y-%m-%d %H:%M:%S')
			end_time = datetime.strptime(res['end_time'], '%Y-%m-%d %H:%M:%S')
			if end_time > reservation_time:
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

		# Take first available slot and adapt the end time to the required end time
		first_available_slot = free_slots[0]
		first_available_slot = (first_available_slot[0], slot[1])

		# Compute delay
		delay = first_available_slot[0] - slot[0]
		return first_available_slot, delay

	def reserve_slot(self, node, slot, comm):

		# Assertions
		assert isinstance(node, str)
		assert isinstance(slot[0], datetime)
		assert isinstance(slot[1], timedelta)
		assert isinstance(comm, Comm)

		# Get slot
		reservation_start_time = slot[0]
		reservation_end_time = reservation_start_time + slot[1]

		# Get all reservations
		reservations = comm.sql_select_reservations('environmental_agents', node,  self.agv.id)
		
		# Check if reservation is valid before reserving
		valid = True
		for res in reservations:
			start_time = datetime.strptime(res['start_time'], '%Y-%m-%d %H:%M:%S')
			end_time = datetime.strptime(res['end_time'], '%Y-%m-%d %H:%M:%S')
			if (reservation_start_time < start_time < reservation_end_time) or (reservation_start_time < end_time < reservation_end_time) or (start_time < reservation_start_time and end_time > reservation_end_time):
				valid = False
				break
		if valid:
			reservation_dict = {'node': node, 'robot': self.agv.id, 'start_time': reservation_start_time.strftime('%Y-%m-%d %H:%M:%S'), 'end_time': reservation_end_time.strftime('%Y-%m-%d %H:%M:%S'), 'pheromone': 100.0}
			comm.sql_add_to_table('environmental_agents', reservation_dict)
			return True
		else:
			print("Could not add slot (" + str(reservation_start_time) + ', ' + str(reservation_end_time) + ") of agv " + str(self.agv.id))
			return False
