from datetime import datetime, timedelta
import numpy as np

from robotino_core.solvers.tsp_solver import tsp
from robotino_core.solvers.astar_solver import get_shortest_path
from robotino_core.solvers.astar_solver import get_alternative_paths
from robotino_core.Comm import Comm

def dmas(start_node, nodes_to_visit, start_time, graph, id, speed, comm):

	"""
        Implements the dmas algorithm to find the best of a set of alternative routes with available slots. The set of feasible routes contains the 
		shortest A* path to the destination node or the shortest A* path through multiple destination nodes. The set of feasible routes is augmented 
		with alternative routes obtained by a ACO solver. 
        Input:
            - Start node name (str)
            - Nodes to visit names (list)
            - Start time (datetime)
			- Graph
			- Robot id (int)
			- Robot speed (float)
			- Number of required alternative paths
			- Communication channel
        Output:
            - Best route (without starting node)
            - Best slots (without starting node)
            - Total distance in meters
			- Total cost in seconds
        Default output:
            - []
            - [(start_time, timedelta())]
            - 0
			- 0
    """

	# Assertions
	assert isinstance(start_node, str)
	assert isinstance(nodes_to_visit, list)
	assert isinstance(start_time, datetime)
	assert isinstance(id, int)
	assert isinstance(speed, float)
	assert isinstance(comm, Comm)

	# Init
	feasible_paths = []

	# If start and end are the same
	if start_node in nodes_to_visit: return [], [(start_time, timedelta())], 0, 0

	# Define distance function
	def dist_func(a, b):
		path, dist = dist_astar(graph, a, b)
		return path, dist

	# Compute tsp solution
	solution = tsp(start_node, nodes_to_visit, dist_func)
	tsp_path = [start_node] + [item for sublist in solution['paths'] for item in sublist]
	feasible_paths.append(tsp_path)

	# Get alternative paths
	_, local_best_solutions = get_alternative_paths(graph, start_node, nodes_to_visit[0], 2)
	feasible_paths.extend(local_best_solutions)

	# Exploration ants
	fitness_values, slots, dists, costs = explore(local_best_solutions, start_time, graph, id, speed, comm)

	# Best route selection
	best_path = local_best_solutions[int(np.argmin(fitness_values))]
	best_slots = slots[int(np.argmin(fitness_values))]
	best_dist = dists[int(np.argmin(fitness_values))]
	best_cost = costs[int(np.argmin(fitness_values))]

	return best_path[1:], best_slots, best_dist, best_cost

def explore(paths, start_time, graph, id, speed, comm):

	# Assertions
	assert isinstance(paths, list)
	assert isinstance(start_time, datetime)
	assert isinstance(id, int)
	assert isinstance(speed, float)
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
		for i in range(len(path) - 1):
			
			# Calculate dist and traveltime to drive to node i+1
			dist = graph.edges[path[i], path[i + 1]].length
			travel_time = timedelta(seconds= dist / speed)
			
			# Get wanted slot
			wanted_slot = (timestamp, travel_time)
			
			# Check available slots for node i+1
			slot, delay = check_slot(path[i+1], wanted_slot, id, comm)
			
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

def intent(path, slots, id, comm):
	
	# Assertions
	try:
		assert isinstance(path, list)
		assert isinstance(slots, list)
		assert isinstance(id, int)
		assert isinstance(comm, Comm)
	except:
		print("Error in intent")
		print(path)
		print(slots)
		exit()
	
	# Intent
	for i in range(len(path)):
		result = reserve_slot(path[i], slots[i], id, comm)
		if not result:
			return False
	return True
		
def check_slot(node, slot, id, comm):
		
	# Assertions
	assert isinstance(node, str)
	assert isinstance(slot[0], datetime)
	assert isinstance(slot[1], timedelta)
	assert isinstance(comm, Comm)

	# Get slot information
	reservation_time = slot[0]
	duration = slot[1]

	# Get all reservations
	reservations = comm.sql_select_reservations('environmental_agents', node,  id)

	# If database not alive
	if reservations is None: return None, None
	
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

def reserve_slot(node, slot, id, comm):

	# Assertions
	assert isinstance(node, str)
	assert isinstance(slot[0], datetime)
	assert isinstance(slot[1], timedelta)
	assert isinstance(comm, Comm)
		
	# Get slot
	reservation_start_time = slot[0]
	reservation_end_time = reservation_start_time + slot[1]
	
	# Get all reservations
	reservations = comm.sql_select_reservations('environmental_agents', node,  id)

	# If database not alive
	if reservations is None: return False
	
	# Check if reservation is valid before reserving
	valid = True
	for res in reservations:
		start_time = datetime.strptime(res['start_time'], '%Y-%m-%d %H:%M:%S')
		end_time = datetime.strptime(res['end_time'], '%Y-%m-%d %H:%M:%S')
		if (reservation_start_time < start_time < reservation_end_time) or (reservation_start_time < end_time < reservation_end_time) or (start_time < reservation_start_time and end_time > reservation_end_time):
			valid = False
			break
	if valid:
		reservation_dict = {'node': node, 'robot': id, 'start_time': reservation_start_time.strftime('%Y-%m-%d %H:%M:%S'), 'end_time': reservation_end_time.strftime('%Y-%m-%d %H:%M:%S'), 'pheromone': 100.0}
		res = comm.sql_add_to_table('environmental_agents', reservation_dict)
		return True if res else False
	else:
		print("Could not add slot (" + str(reservation_start_time) + ', ' + str(reservation_end_time) + ") of agv " + str(id))
		return False

def dist_astar(graph, start_node, end_node):
	path, dist = get_shortest_path(graph, start_node, end_node)
	return path[1:], dist

def dist_euclidean(graph, start_node, end_node):
	dist = np.linalg.norm(np.subtract(graph.nodes[start_node].pos, graph.nodes[end_node].pos))
	return [], dist
