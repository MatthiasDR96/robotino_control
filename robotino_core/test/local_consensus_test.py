import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
from robotino_core.Graph import Graph
from robotino_core.solvers.tsp_solver import tsp

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

def compute_bid(local_task_list, task, substract, start_node):

		# Get tasks in new local task list
		if not substract:
			new_local_task_list = (local_task_list if local_task_list is not None else []) + [task]
		else:
			new_local_task_list = [task_ for task_ in local_task_list if task_['id'] != task['id']]

		# Get start node of robot
		start_node = start_node

		# Compute cost of current tour
		_, _, current_cost = compute_task_sequence_and_cost(local_task_list, start_node)

		# Compute cost of new tour
		new_sequence, new_edges, new_cost = compute_task_sequence_and_cost(new_local_task_list, start_node)

		# Marginal cost to execute task
		min_sum = new_cost - current_cost
		min_max = new_cost

		# Objective list
		objective_list = [min_sum, min_max]

		return objective_list

def compute_task_sequence_and_cost(task_list, start_node):

		nodes_to_visit = [task['node'] for task in task_list]
		task_sequence_, _, edges = tsp(graph, start_node, nodes_to_visit)
		task_sequence = [task_list[nodes_to_visit.index(name)] for name in task_sequence_]
		edges = np.array(edges)
		cost = sum(edges)

		return task_sequence, edges, cost

def resolution_lb(bids):
		bids.sort(key=lambda p: p['values'][0]) # Minimize start time of task
		best_bid = bids[0]
		return best_bid	
		
if __name__ == "__main__":
	
	# Get all robots in the fleet
	local_task_list_1 = []
	local_task_list_2 = [{'id': 1, 'node': 'pos_6'}, {'id': 2, 'node': 'pos_7'}, {'id': 3, 'node': 'pos_8'}]

	# Task to exchange
	task = 0

	# Compute bid for accepting task (robot 1)
	best_bid = compute_bid(local_task_list_1, local_task_list_2[task], False, 'pos_2')[0]

	# Compute bid for removing task (robot 2)
	my_bid = compute_bid(local_task_list_2, local_task_list_2[task], True, 'pos_3')[0]

	# Choose
	if best_bid + my_bid < 0:
		print(True)
	else:
		print(False)

	# Print
	print(best_bid)
	print(my_bid)
	plt.show()