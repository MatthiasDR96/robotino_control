import numpy as np
from datetime import datetime, timedelta
from robotino_core.Comm import Comm
from robotino_core.solvers.astar_solver import find_shortest_path
from robotino_core.solvers.feasibility_ant_solver import feasibility_ant_solve

class World:

	def __init__(self, nodes):
		self.nodes = nodes
		self.edges = self.create_edges()

	def create_edges(self):
		edges = {}
		for m in self.nodes:
			for n in self.nodes:
				if m != n:
					edge = Edge(m, n)
					edges[m, n] = edge
		return edges

	def reset_pheromone(self, level=0.01):
		for edge in self.edges.values():
			edge.pheromone = level

class Edge:

	def __init__(self, start, end, pheromone=None):
		self.start = start
		self.end = end
		self.possible_routes = []  #TODO ASTAR 
		self.pheromone = 0.1 if pheromone is None else pheromone

	def __eq__(self, other):
		if isinstance(other, self.__class__):
			return self.__dict__ == other.__dict__
		return False

def check_slot(node, slot, comm):

		# Assertions
		assert isinstance(node, str)
		assert isinstance(slot[0], datetime)
		assert isinstance(slot[1], timedelta)

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

def objective_function(start_time, start_node, start_battery, graph, sequence, charging_index, charging_station, charging_time):

	# Create sequence including charging station
	new_sequence = [sequence_to_optimize[i] for i in sequence].insert(charging_index, charging_station)

	# Start conditions
	start_time = start_time
	start_node = start_node
	start_battery = start_battery

	# Performance attributes
	dist = 0.0
	cost = 0.0
	charging_cost = 0.0
	charging_time = 0.0
	paths = {}
	slots = {}

	# Loop
	for task in new_sequence:

		### Calculate route ###

		# Find feasible paths
		local_best_solutions = find_shortest_path(graph, start_node, task['node']) ## TODO astar

		# Init
		fitness_values = []
		total_travel_costs = []
		total_delays = []
		all_slots = []

		# Loop
		for path in local_best_solutions:

			# Init
			timestamp = start_time
			total_delay = timedelta()
			total_travel_time = timedelta()
			slots = []

			# Calculate slot of nodes in between
			for i in range(0, len(path) - 1):

				# Calculate traveltime to drive to node i+1
				travel_dist = graph.edges[path[i], path[i + 1]].length
				travel_time = timedelta(seconds = travel_dist / 1.0)
				wanted_slot = (timestamp, travel_time)

				# Calculate battery cost to drive to node i+1
				start_battery -= self.resource_scale_factor * travel_time

				# Check available slots for node i+1
				slot, delay = check_slot(path[i+1], wanted_slot, comm)

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

		# Best route selection
		best_path = local_best_solutions[int(np.argmin(fitness_values))][1:]
		best_slots = slots[int(np.argmin(fitness_values))]

		### Calculate task costs ###

		if task['message'] == 'charging':
			charging_cost += 0.0
			charging_time += 0.0
		else:
			cost += 0.0 # For handling time

		# Update start attributes
		start_time = best_slots[-1][0] + best_slots[-1][1]
		start_node = best_path[-1]

		# Update performance_attributes
		dist += travel_dist
		cost += total_travel_costs
		charging_cost += 0.0
		charging_time += 0.0
		paths[task['id']] = best_path
		slots[task['id']] = best_path


# Comm
id = 15
comm = Comm('localhost', 10000+id, id, 'localhost', 'matthias', 'matthias', 'kb')
comm.sql_open()

# Get graph
graph = comm.get_graph()

# Situation
local_task_list = comm.sql_get_local_task_list(id)
tasks_to_assign = comm.sql_select_from_table('global_task_list', 'status', "unassigned")
new_task = tasks_to_assign[0]

# Start conditions
start_node = 'pos_1'
start_time = datetime.now()
start_battery = 100
sequence_to_optimize = local_task_list + [new_task]

# Possible solution
sequence = [0, 1, 2]
charging_index = 1
charging_station = 1
charging_time = 10

def distance_function(m, n):
	 # Dmas
     best_path, best_slots, best_dist = dmas(m, n, self.start_time, self.comm)
	 
# World
world = World(graph, [start_node] + sequence_to_optimize, distance_function)






