import yaml
import os
import matplotlib.pyplot as plt
from datetime import datetime

from robotino_core.solvers.tsp_solver import *
from robotino_core.solvers.astar_solver import *
from robotino_core.Graph import Graph
from robotino_core.Comm import Comm
from robotino_core.solvers.dmas_solver import *
from robotino_core.agv.AGV_RM_agent import RM_agent
from robotino_core.agv.AGV_Main import AGV

import matplotlib.pyplot as plt

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

def search_closest_charging_station(location):
		closest_point = None
		min_distance = float('inf')
		for name in ['pos_1', 'pos_2', 'pos_3']: #self.agv.params['depot_locations']:
			_, distance = get_shortest_path(graph, location, name)
			if distance < min_distance:
				min_distance = distance
				closest_point = name
		return closest_point

def objective_function_insertion(insertion_index, start_task, start_time, task_sequence, initial_resources):

	# Start node
	start_task = start_task if insertion_index == 0 else task_sequence[insertion_index - 1]

	# End node
	end_task = task_sequence[insertion_index]

	# Closest charging station
	charging_station = search_closest_charging_station(start_task)

	# New task sequence
	new_task_sequence = copy(task_sequence)
	new_task_sequence.insert(insertion_index, charging_station)

	# Initial cost from start to end node
	cost_1 = local_task_list_estimated_cost[end_task]

	# New tour
	_, cost_2 = get_shortest_path(graph, start_task, charging_station)
	_, cost_3 = get_shortest_path(graph, charging_station, end_task)

	# Cost to travel to charging station
	charging_cost_travel = cost_2 + cost_3 - cost_1

	# Compute cost before and after charging
	tasks_before_charging = task_sequence[:insertion_index]
	tasks_after_charging = task_sequence[insertion_index+1:]
	cost_before_charging = cost_2 + sum([local_task_list_estimated_cost[task] for task in tasks_before_charging])
	cost_after_charging = cost_3 + sum([local_task_list_estimated_cost[task] for task in tasks_after_charging])

	# Charge level at nodes
	delta_before_charging = cost_before_charging * resource_scale_factor
	delta_after_charging = cost_after_charging * resource_scale_factor

	# Compute charging cost
	charging_delta = delta_after_charging + battery_threshold - (initial_resources - delta_before_charging)
	charging_cost_loading = charging_delta / resource_scale_factor

	# Cannot load more than 100%
	statement_1 = initial_resources - delta_before_charging >= 20
	statement_2 = initial_resources - delta_before_charging + charging_delta <= 100
	statement_3 = initial_resources - delta_before_charging + charging_delta - delta_after_charging >= 20
	statement_4 = charging_delta >= 0 and charging_delta <= 100
	statement_5 = initial_resources - sum([local_task_list_estimated_cost[task] for task in task_sequence]) * resource_scale_factor < 20
	if not statement_1 or not statement_2 or not statement_3 or not statement_4 or not statement_5:
		return float('inf'), None, None

	# Calculate fitness as the total charging cost
	fitness = charging_cost_travel + charging_cost_loading

	#print("Start node: " + start_task)
	#print("End node: " + end_task)
	#print("Charging station: " + charging_station)
	#print("New task sequence: " + str(new_task_sequence))
	#print("Cost 1: " + str(cost_1))
	#print("Cost 2: " + str(cost_2))
	#print("Cost 3: " + str(cost_3))
	#print("Tasks before charging: " + str(tasks_before_charging))
	#print("Tasks after charging: " + str(tasks_after_charging))
	#print("Cost before charging: " + str(cost_before_charging))
	#print("Cost after charging: " + str(cost_after_charging))
	#print()
	#print("Initial level: " + str(initial_resources))
	#print("Charging delta: " + str(charging_delta))
	#print("Fitness: " + str(fitness))

	return fitness, charging_station, charging_delta

def optimize_brute_force(start_task, task_sequence, initial_resources):

		# Solve
		best_fitness = float('inf')
		best_insertion_index = None
		best_charging_percent = None
		for i in range(len(task_sequence)):
			fitness, charging_station, charging_percent = objective_function_insertion(i, start_task, None, task_sequence, initial_resources)
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


# Comm
comm = Comm('localhost', 10015, 'localhost', 'matthias', 'matthias', 'kb')
comm.sql_open()

# Add tasks to local task list
for task in ['pos_5', 'pos_14']:
	task_dict = {"node": task, "priority": 0, "robot": 15, "message": 'charging', "status": 'assigned'}
	comm.sql_add_to_table('global_task_list', task_dict)

# Situation
local_task_list_estimated_cost = {'pos_8': 60, 'pos_4': 80, 'pos_14': 60, 'pos_16': 80}
resource_scale_factor = 0.1
battery_threshold = 20

# Opt params
insertion_index = 1
start_task = 'pos_2'
start_time = datetime.now()
task_sequence = ['pos_8', 'pos_4', 'pos_14', 'pos_16']
initial_resources = 80

for level in [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]:

	# Compute optimal insertion point, charging station, and charging percent
	charging_station, insertion_index, charging_delta = optimize_brute_force(start_task, task_sequence, level)

	print()
	print("Level: " + str(level))
	print(level - sum([local_task_list_estimated_cost[task] for task in task_sequence]) * resource_scale_factor)
	print("Charging station: " + str(charging_station))
	print("Insertion index: " + str(insertion_index))
	print("Charging_delta: " + str(charging_delta))

# Show
plt.show()