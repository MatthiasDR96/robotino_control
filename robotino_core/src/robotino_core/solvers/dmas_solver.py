from datetime import datetime, timedelta
import numpy as np

from robotino_core.solvers.feasibility_ant_solver import *

def dmas(graph, start, dest, start_time, comm):

	# Assertions
	assert isinstance(start, str)
	assert isinstance(dest, str)
	assert isinstance(start_time, datetime)

	# Feasibility ants
	feasible_paths, feasible_distances = get_alternative_paths(graph, start, dest, 5)

	# Exploration ants
	fitness_values, slots = explore(graph, feasible_paths, start_time, comm)

	# Best route selection
	best_path = feasible_paths[int(np.argmin(fitness_values))]
	best_slots = slots[int(np.argmin(fitness_values))]
	best_dist = feasible_distances[int(np.argmin(fitness_values))]

	return best_path[1:], best_slots, best_dist

def explore(graph, paths, start_time, comm):

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
			travel_time = timedelta(seconds=graph.edges[path[i], path[i + 1]].dist / 1.0)
			wanted_slot = (timestamp, travel_time)

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

	return fitness_values, all_slots

def check_slot(node, slot, comm):

		# Assertions
		assert isinstance(node, str)
		assert isinstance(slot[0], datetime)
		assert isinstance(slot[1], timedelta)

		# Get slot information
		reservation_time = slot[0]
		duration = slot[1]

		# Get all reservations
		reservations = comm.sql_select_reservations('environmental_agents', node,  15)

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
