from pyswarm import pso

from robotino_core.Comm import Comm
from robotino_core.solvers.genetic_algorithm_solver import *
from robotino_core.solvers.tsp_solver import *

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

		# Temporal optimization params
		self.candidate_resource_percent = None
		self.initial_resources = None
		self.initial_task_sequence = None
		self.candidate_task_sequence = None
		self.candidate_insertion_index = None
		self.start_node = None

	def main(self):

		# Loop
		print("   Resource agent:            Started")
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
					self.comm_main.sql_add_to_table('global_task_list', task_dict)

	def solve(self, task_sequence):

		# Init
		charging_station = None
		insertion_index = None
		charging_time = 0
		charging_cost = 0
		if not len(task_sequence) == 0:

			# Calculate initial resources at start node
			if self.agv.task_executing:
				_, distance = find_shortest_path(self.agv.kb['graph'], self.agv.robot_node,
												 self.agv.task_executing.pos_A)
				if self.agv.task_executing.order_number == '000':
					init_resources = self.agv.battery_status - self.resource_consumption(
						distance / self.agv.robot_speed) + self.agv.task_executing.priority
				else:
					init_resources = self.agv.battery_status - self.resource_consumption(
						distance / self.agv.robot_speed)
			else:
				init_resources = self.agv.battery_status

			# Start node
			start_node = self.agv.task_executing.pos_A if self.agv.task_executing else self.agv.robot_node
			self.start_node = start_node

			# Calculate resources at each node without charging
			resources_at_nodes = self.calculate_resources_at_nodes(task_sequence, start_node, init_resources, None, 0)

			# Insert charging station if needed
			if resources_at_nodes[-1] <= self.agv.battery_threshold:

				# Compute optimal insertion point, charging station, and charging percent
				charging_station, insertion_index, charging_percent = self.optimize_brute_force(task_sequence,
																								start_node,
																								init_resources)
				if not charging_station:
					raise Exception('agv can not reach a solution bc of battery limits')

				charging_time = charging_percent / self.charging_factor

				# New task sequence
				new_task_sequence = copy(task_sequence)
				new_task_sequence.insert(insertion_index, charging_station)

				# Start node
				start_node = start_node if insertion_index == 0 else new_task_sequence[insertion_index - 1]
				end_node = new_task_sequence[insertion_index + 1]
				charging_station = self.search_closest_charging_station(start_node)
				_, dist1 = find_shortest_path(self.agv.kb['graph'], start_node, end_node)
				_, dist2 = find_shortest_path(self.agv.kb['graph'], start_node, charging_station)
				_, dist3 = find_shortest_path(self.agv.kb['graph'], charging_station, end_node)
				charging_cost_travel = dist2 + dist3 - dist1
				charging_cost = charging_cost_travel + charging_time

		return charging_station, insertion_index, charging_time, charging_cost

	def calculate_resources_at_nodes(self, task_sequence, start_node, initial_resources, charging_index,
									 charging_percent):

		# From robot node to first task
		charging_index_tour = -1
		tour, _ = find_shortest_path(self.agv.kb['graph'], start_node, task_sequence[0])
		for i in range(len(task_sequence) - 1):
			if i == charging_index:
				charging_index_tour = len(tour) - 1
			path, _ = find_shortest_path(self.agv.kb['graph'], task_sequence[i], task_sequence[i + 1])
			tour += path[1:]

		# Compute normal consumption
		resources_at_nodes = np.zeros(len(tour))
		resources_at_nodes[0] = initial_resources
		for i in range(len(tour) - 1):
			# Get nodes
			node = tour[i]
			next_node = tour[i + 1]

			# Calculate consumed resources to drive to next station
			distance = self.agv.kb['graph'].edges[node, next_node].length
			travel_time = distance / self.agv.robot_speed
			consumed_resources = self.resource_consumption(travel_time)

			# Calculate total resources at node
			resources_at_nodes[i + 1] = resources_at_nodes[i] - consumed_resources

		# Add charged resources
		resources_at_nodes[charging_index_tour:] += charging_percent

		return resources_at_nodes

	def search_closest_charging_station(self, location):
		closest_point = None
		min_distance = float('inf')
		for name in self.agv.params['depot_locations']:
			_, distance = find_shortest_path(self.agv.graph, location, name)
			if distance < min_distance:
				min_distance = distance
				closest_point = name
		return closest_point

	def resource_consumption(self, travel_time):
		charge_loss = self.resource_scale_factor * travel_time
		return charge_loss

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

	def optimize_brute_force(self, task_sequence, start_node, initial_resources):

		# Set this to be used in fitness function
		self.initial_resources = initial_resources
		self.initial_task_sequence = task_sequence

		# Solve
		best_fitness = float('inf')
		best_solution = None
		best_charging_percent = None
		for i in range(len(task_sequence)):
			fitness = self.objective_function_insertion(i)
			if fitness < best_fitness:
				best_fitness = fitness
				best_solution = i
				best_charging_percent = self.candidate_resource_percent

		# Get output
		if best_fitness == float('inf') or best_fitness == 1e+100:
			return None, None, None
		else:
			insertion_index = best_solution
			start_node = start_node if insertion_index == 0 else self.initial_task_sequence[insertion_index - 1]
			charging_station = self.search_closest_charging_station(start_node)
			return charging_station, insertion_index, best_charging_percent

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

	def objective_function_insertion(self, x):

		# Constraint
		fitness = float('inf')

		# Get optimization params
		insertion_index = x  # int("".join(str(x_) for x_ in x), 2)
		self.candidate_insertion_index = insertion_index

		if insertion_index < len(self.initial_task_sequence):
			# Start node
			start_node = self.start_node if insertion_index == 0 else self.initial_task_sequence[insertion_index - 1]

			# End node
			end_node = self.initial_task_sequence[insertion_index]

			# Closest charging station
			charging_station = self.search_closest_charging_station(start_node)

			# New task sequence
			new_task_sequence = copy(self.initial_task_sequence)
			new_task_sequence.insert(insertion_index, charging_station)
			self.candidate_task_sequence = new_task_sequence

			# Initial tour
			_, dist1 = find_shortest_path(self.agv.kb['graph'], start_node, end_node)

			# New tour
			_, dist2 = find_shortest_path(self.agv.kb['graph'], start_node, charging_station)
			_, dist3 = find_shortest_path(self.agv.kb['graph'], charging_station, end_node)

			# Cost to travel to charging station
			charging_cost_travel = dist2 + dist3 - dist1

			# Compute charging cost
			opt_resource_percent, fitness = self.optimize_resource_percent()
			self.candidate_resource_percent = opt_resource_percent

			# Calculate fitness as the total charging cost
			fitness = fitness + charging_cost_travel

		return fitness

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
		if min_resource < self.agv.battery_threshold:
			punishment = float('inf')
		# Resources cannot raise above max resources
		if max_resource > 100:
			punishment = float('inf')

		return x * self.charging_factor + punishment

	def compute_tour(self, task_sequence):
		tour, cost = find_shortest_path(self.agv.kb['graph'], self.start_node, task_sequence[0])
		for i in range(len(task_sequence) - 1):
			path, distance = find_shortest_path(self.agv.kb['graph'], task_sequence[i], task_sequence[i + 1])
			tour += path
			cost += distance
		return tour, cost


def calculate_travel_cost(route, graph, speed):
	path_cost = 0
	for i in range(len(route) - 1):
		distance = graph.edges[route[i], route[i + 1]].length
		travel_time = distance / speed
		path_cost += travel_time
	return path_cost
