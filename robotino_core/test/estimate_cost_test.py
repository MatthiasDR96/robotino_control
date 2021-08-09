import math
import yaml
import os
import matplotlib.pyplot as plt
from datetime import datetime, timedelta

from robotino_core.Graph import Graph
from robotino_core.agv.AGV_Routing import Routing
from robotino_core.agv.AGV_Main import AGV
from robotino_core.Comm import Comm
from robotino_core.solvers.astar_solver import find_shortest_path

def calculate_euclidean_distance(a, b):
	return math.sqrt(math.pow(b[0] - a[0], 2) + math.pow(b[1] - a[1], 2))

def cost_estimator(executing_task, local_task_list, current_node, current_loc, start_time, routing):

	# Start point
	current_loc = current_loc
	current_node = current_node

	# Outputs
	task_slots = []
	total_cost = timedelta()

	if routing:

		# Evaluate executing task
		end_time = reserved_slots[executing_task][-1][0] + reserved_slots[executing_task][-1][1]
		duration = end_time - start_time

		# Update executing task
		task_slots.append((start_time, end_time, duration))
		total_cost += duration

		# Evaluate local task list
		for task in local_task_list:
			start_time = reserved_slots[task][0][0]
			end_time = reserved_slots[task][-1][0] + reserved_slots[task][-1][1]
			duration = end_time - start_time
			task_slots.append((start_time, end_time, duration))
			total_cost += duration

	else:

		# Evaluate executing task
		dist = calculate_euclidean_distance((current_loc[0], current_loc[1]), graph.nodes[current_node].pos)
		dist += find_shortest_path(graph, current_node, executing_task)[1]
		cost = timedelta(seconds=dist / 1.0)

		# End time
		end_time = start_time + cost
		duration = end_time - start_time

		# Update executing task
		task_slots.append((start_time, end_time, duration))
		total_cost += cost

		# Evaluate local task list
		start_time = end_time
		start_node = executing_task
		for task in local_task_list:
			dist = find_shortest_path(graph, start_node, task)[1]
			cost = timedelta(seconds=dist / 1.0)
			end_time = start_time + cost
			duration = end_time - start_time
			task_slots.append((start_time, end_time, duration))
			total_cost += cost
			start_time = end_time
			start_node = task

	return task_slots, total_cost
		
if __name__== "__main__":

	# Params
	robot_id = 15

	# Start time
	start_time_init = datetime(2000,1,1,0,0,0,0)

	# AGV
	agv = AGV('localhost', 10015, 15, 'localhost', 'matthias', 'matthias', 'kb', (10, 30), False, False, False)

	# Open connection to database
	comm = Comm('localhost', 10015, 'localhost', 'matthias', 'matthias', 'kb')
	comm.sql_open()

	# Calculate estimated end time and duration
	routing = Routing(agv)

	# Read params
	this_dir = os.path.dirname(os.path.dirname(__file__))
	data_path = os.path.join(this_dir, "src/robotino_core/params", "setup.yaml")
	with open(data_path, 'r') as file:
		params = yaml.load(file, Loader=yaml.FullLoader)

	# Create graph
	graph = Graph()
	node_neighbors = params['node_neighbors']
	node_locations = params['node_locations']
	graph.create_nodes(list(node_locations.values()), list(node_locations.keys()))
	graph.create_edges(list(node_neighbors.keys()), list(node_neighbors.values()))
	graph.plot()

	# Current loc
	current_loc = (20, 30)

	# Current node
	current_node = 'pos_5'

	# Executing_task
	executing_task = 'pos_6'

	# Get tasks in local task list
	local_task_list = ['pos_11', 'pos_14','pos_16']

	# Dmas update
	reserved_paths = {}
	reserved_slots  = {}
	total_path = []

	# Update paths for executing task
	start_time = start_time_init + timedelta(seconds=calculate_euclidean_distance((current_loc[0], current_loc[1]), graph.nodes[current_node].pos) / 1.0)
	best_path, best_slots = routing.dmas(current_node, [executing_task], start_time, comm)
	reserved_paths[executing_task] = best_path
	reserved_slots[executing_task] = best_slots

	# Update paths for each task in local task list
	estimated_start_time = best_slots[-1][0] + best_slots[-1][1]
	start_node = executing_task
	for task in local_task_list:

		# Do dmas
		best_path, best_slots = routing.dmas(start_node, [task], estimated_start_time, comm)

		# If a solution exist
		if best_path:

			# Set total planned path
			total_path.extend(best_path)

			# Estimated end time and duration
			estimated_end_time = best_slots[-1][0] + best_slots[-1][1]
			estimated_duration = estimated_end_time - estimated_start_time

			# Set paths towards all tasks
			reserved_paths[task] = best_path
			reserved_slots[task] = best_slots
			
			# Estimated start time for next task
			estimated_start_time = estimated_end_time

			# Start node for next task is end node of previous task
			start_node = task

	# Print
	print()
	for key, value in reserved_paths.items():
		print(key + ': ' + str(value))
	print()
	for key, value in reserved_slots.items():
		print(key + ': ' + str(value))

	# Evaluate current route
	task_slots, cost = cost_estimator(executing_task, local_task_list, current_node, current_loc, start_time_init, False)

	# Print results
	print()
	print("Astar: ")
	for task in task_slots:
		print(task)
	print(cost.total_seconds())

	# Evaluate current route
	task_slots, cost = cost_estimator(executing_task, local_task_list, current_node, current_loc, start_time_init, True)
	
	# Print results
	print()
	print("Dmas: ")
	for task in task_slots:
		print(task)
	print(cost.total_seconds())

	# Plot graph
	plt.show()

