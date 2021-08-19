import yaml
import os
import matplotlib.pyplot as plt

from robotino_core.solvers.tsp_solver import *
from robotino_core.solvers.astar_solver import *
from robotino_core.Graph import Graph

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

# New task
new_task = 'pos_4'

# Get tasks in local task list
local_task_list = ['pos_14', 'pos_11']

# Get tasks in new local task list
new_local_task_list = local_task_list + [new_task]

# Get start node of robot
start_node = 'pos_1'

# Distance function
def distance_function(start_node, end_node):
    nodes_in_between, length = get_shortest_path(graph, start_node, end_node)
    return nodes_in_between, [], length, length

# Compute cost of current tour
current_solution = tsp(start_node, local_task_list, distance_function)
current_cost = sum(current_solution['costs'])

# Compute cost of new tour
new_solution = tsp(start_node, new_local_task_list, distance_function)
new_cost = sum(new_solution['costs'])

# Marginal cost to execute task
min_sum = new_cost - current_cost
min_max = new_cost

# Start time of new task
task_index = new_solution['sequence'].index(new_task)
start_time = sum(new_solution['costs'][0:task_index])
end_time = sum(new_solution['costs'][0:task_index+1])
duration = end_time - start_time

# Results
print("Curent task sequence: " + str(current_solution['sequence']))
print("Curent task paths: " + str(current_solution['paths']))
print("Curent task costs: " + str(current_solution['costs']))
print("New task sequence: " + str(new_solution['sequence']))
print("New task paths: " + str(new_solution['paths']))
print("New task costs: " + str(new_solution['costs']))
print("Min sum: " + str(min_sum))
print("Min max: " + str(min_max))
print("Start time: " + str(start_time))
print("End time: " + str(end_time))
print("Duration: " + str(duration))
plt.show()