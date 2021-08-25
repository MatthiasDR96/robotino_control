from pyswarm import pso
from copy import copy
import time

from robotino_core.Comm import Comm
from robotino_core.solvers.genetic_algorithm_solver import *
from robotino_core.solvers.tsp_solver import *
from robotino_core.solvers.astar_solver import *

class RM_agent:
	"""
			A class containing the intelligence of the Resource Management agent
	"""

	def __init__(self, agv):

		# AGV
		self.agv = agv

		# Init database communication
		self.comm_main = Comm(self.agv.ip, self.agv.port, self.agv.host, self.agv.user, self.agv.password, self.agv.database)
		self.comm_main.sql_open()
		self.comm_threshold_charging = Comm(self.agv.ip, self.agv.port, self.agv.host, self.agv.user, self.agv.password, self.agv.database)
		self.comm_threshold_charging.sql_open()

		# Temporal optimization params
		self.candidate_resource_percent = None
		self.initial_resources = None
		self.initial_task_sequence = None
		self.candidate_task_sequence = None
		self.candidate_insertion_index = None
		self.start_node = None

	def main(self, _, stop):

		# Loop
		print("RM-agent:	Main thread started")
		while True:

			# Timeout
			time.sleep(self.agv.params['dmas_rate']+1)

			# Get start node and local task list
			start_task = self.agv.task_executing
			home_task = {"node": self.agv.depot}
			local_task_list = [task for task in self.comm_main.sql_get_local_task_list(self.agv.id) if not task['message'] == 'charging'] 

			# Task executing cost
			task_executing_cost = self.agv.task_executing_estimated_cost

			# Local task list cost
			local_task_list_cost = self.agv.local_task_list_estimated_cost

			# If path already planned
			if not all(elem in local_task_list_cost.keys() for elem in [task['id'] for task in local_task_list]):
				continue

			# Compute end resource level
			task_executing_charge_loss = self.resource_consumption(task_executing_cost)
			local_task_list_charge_loss = [self.resource_consumption(cost) for cost in local_task_list_cost.values()]
			total_charge_loss = task_executing_charge_loss + sum(local_task_list_charge_loss)
			end_level = self.agv.battery_status - total_charge_loss

			# print()
			# print("Executing task cost: " + str(task_executing_cost))
			# print("Local task list cost: " + str(local_task_list_cost))
			# print("Executing task delta: " + str(task_executing_charge_loss))
			# print("Local task list delta: " + str(local_task_list_charge_loss))
			# print("Total charge loss: " + str(total_charge_loss))
			# print("End level: " + str(end_level))

			# Solve for optimal charging station
			if not end_level <= self.agv.params['battery_threshold']:
				continue

			# Compute optimal insertion point, charging station, and charging percent
			charging_station, insertion_index, charging_percent = self.optimize_brute_force(local_task_list, start_task, self.agv.battery_status)

			print("Charging station: " + str(charging_station))
			print("Insertion index: " + str(insertion_index))
			print("Charging percent: " + str(charging_percent))

			# If no need to charge:
			if insertion_index is None:
				continue

			# Add or update charging station
			charging_station = [task for task in local_task_list if task['message'] == 'charging']
			if len(charging_station) == 0:
				task_dict = {"node": charging_station, "priority": insertion_index, "robot": self.agv.id, "message": 'charging', "status": 'assigned'}
				print(" Add " + str(task_dict))
				self.comm_main.sql_add_to_table('global_task_list', task_dict)
			else:
				task_dict = {"node": charging_station[0]['node'], "priority": insertion_index, "robot": self.agv.id, "message": 'charging', "status": 'assigned'}
				self.comm_main.sql_update_task(charging_station[0]['id'], task_dict)

			# Update local_task_list sequence
			for task in local_task_list:
				if task['priority'] >= insertion_index:
					task_dict = {'priority': task['priority'] + 1}
					self.comm_main.sql_update_task(task['id'], task_dict)

			# Close thread at close event 
			if stop():
				print("RM-agent:	Main thread killed")
				break

	def threshold_charging(self, _, stop):

		# Loop
		print("RM-agent:	Threshold charging thread started")
		while True:

			# Check charging status
			if self.agv.status == 'BUSY':
				if self.agv.battery_status < self.agv.params['battery_threshold']:

					# Update status to empty
					self.agv.status = 'EMPTY'

					# Get start node of robot
					start_node = self.agv.task_executing['node'] if not self.agv.task_executing == -1 else self.agv.node

					# Create charging task
					closest_charging_station = self.search_closest_charging_station(start_node)
					task_dict = {"node": closest_charging_station, "priority": 0, "robot": self.agv.id, "message": 'charging', "status": 'assigned'}
					self.comm_threshold_charging.sql_add_to_table('global_task_list', task_dict)

			# Close thread at close event 
			if stop():
				print("RM-agent:	Threshold charging thread killed")
				break

	def optimize_brute_force(self, task_sequence, start_task, initial_resources):

		# Solve
		best_fitness = float('inf')
		best_insertion_index = None
		best_charging_percent = None
		for i in range(len(task_sequence)):
			fitness, charging_station, charging_percent = self.objective_function_insertion(i, start_task, None, task_sequence, initial_resources)
			if fitness < best_fitness:
				best_fitness = fitness
				best_insertion_index = i
				best_charging_station = charging_station
				best_charging_percent = charging_percent

		# Get output
		if best_fitness == float('inf') or best_fitness == 1e+100:
			return None, None, None
		else:
			return best_charging_station, best_insertion_index, best_charging_percent

	def optimize_ga(self, task_sequence, start_node, initial_resources):

		# Set this to be used in fitness function
		self.initial_resources = initial_resources
		self.initial_task_sequence = task_sequence

		# GA solver
		nvars = len([int(x) for x in bin(len(task_sequence) - 1)[2:]])
		algorithm_param = {'max_num_iteration': min(100, len(task_sequence) * 10),
						   'population_size': 2 * len(task_sequence),
						   'mutation_probability': 0.1, 'elit_size': 3, 'max_stall_generations': 10}
		solution, fitness = genetic_algorithm(self.objective_function_insertion, nvars, algorithm_param)

		if fitness == float('inf') or fitness == 1e+100:
			return None, None, None
		else:
			insertion_index = int("".join(str(x_) for x_ in solution), 2)
			start_node = start_node if insertion_index == 0 else self.initial_task_sequence[insertion_index - 1]
			charging_station = self.search_closest_charging_station(start_node)
			return charging_station, insertion_index, self.candidate_resource_percent

	def optimize_resource_percent(self):

		# Bounds
		lb = np.asanyarray([0])
		ub = np.asanyarray([100])

		# Objective
		xopt, fopt = pso(self.objective_function_resource_time, lb=lb, ub=ub, ieqcons=[], f_ieqcons=(lambda x: 0),
						 args=(),
						 kwargs={}, swarmsize=5, omega=0.5, phip=0.5, phig=0.5, maxiter=100, minstep=1e-4,
						 minfunc=1e-4, debug=False)

		return xopt, fopt

	def objective_function_insertion(self, insertion_index, start_task, start_time, task_sequence, initial_resources):

		# Start node
		start_task = start_task if insertion_index == 0 else task_sequence[insertion_index - 1]

		# End node
		end_task = task_sequence[insertion_index]

		# Closest charging station
		charging_station = self.search_closest_charging_station(start_task['node'])

		# New task sequence
		new_task_sequence = copy(task_sequence)
		new_task_sequence.insert(insertion_index, charging_station)

		# Initial cost from start to end node
		cost_1 = self.agv.local_task_list_estimated_cost[end_task['id']]

		# New tour
		_, cost_2 = get_shortest_path(self.agv.graph, start_task['node'], charging_station)
		_, cost_3 = get_shortest_path(self.agv.graph, charging_station, end_task['node'])

		# Cost to travel to charging station
		charging_cost_travel = cost_2 + cost_3 - cost_1

		# Compute cost before and after charging
		tasks_before_charging = task_sequence[:insertion_index]
		tasks_after_charging = task_sequence[insertion_index+1:]
		cost_before_charging = cost_2 + sum([self.agv.local_task_list_estimated_cost[task['id']] for task in tasks_before_charging])
		cost_after_charging = cost_3 + sum([self.agv.local_task_list_estimated_cost[task['id']] for task in tasks_after_charging])

		# Charge level at nodes
		delta_before_charging = self.resource_consumption(cost_before_charging)
		delta_after_charging = self.resource_consumption(cost_after_charging)

		# Compute charging cost
		charging_delta = delta_after_charging + self.agv.params['battery_threshold'] - (initial_resources - delta_before_charging)
		charging_cost_loading = self.resource_consumption_inverse(charging_delta)

		# Cannot load more than 100%
		statement_1 = initial_resources - delta_before_charging >= 20
		statement_2 = initial_resources - delta_before_charging + charging_delta <= 100
		statement_3 = initial_resources - delta_before_charging + charging_delta - delta_after_charging >= 20
		statement_4 = charging_delta >= 0 and charging_delta <= 100
		statement_5 = initial_resources - self.resource_consumption(sum([self.agv.local_task_list_estimated_cost[task['id']] for task in task_sequence])) < 20
		if not statement_1 or not statement_2 or not statement_3 or not statement_4 or not statement_5:
			return float('inf'), None, None

		# Calculate fitness as the total charging cost
		fitness = charging_cost_travel + charging_cost_loading

		return fitness, charging_station, charging_delta

	def resource_consumption(self, cost):
		return cost * self.agv.params['resource_scale_factor']

	def resource_consumption_inverse(self, delta):
		return delta / self.agv.params['resource_scale_factor']

	def objective_function_resource_time(self, x):

		# Evaluate tour
		resources_at_nodes = self.calculate_resources_at_nodes(self.candidate_task_sequence, self.start_node,
															   self.initial_resources,
															   self.candidate_insertion_index, x)

		# Constraints
		punishment = 0
		min_resource = min(resources_at_nodes)
		max_resource = max(resources_at_nodes)
		# Resource cannot be less than the minimum resource level
		if min_resource < self.agv.params['battery_threshold']:
			punishment = float('inf')
		# Resources cannot raise above max resources
		if max_resource > 100:
			punishment = float('inf')

		return x * self.charging_factor + punishment

	def search_closest_charging_station(self, location):
		closest_point = None
		min_distance = float('inf')
		for name in self.agv.params['depot_locations']:
			_, distance = get_shortest_path(self.agv.graph, location, name)
			if distance < min_distance:
				min_distance = distance
				closest_point = name
		return closest_point
