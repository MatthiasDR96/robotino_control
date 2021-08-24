import yaml
import os
import matplotlib.pyplot as plt
from datetime import datetime

from robotino_core.solvers.tsp_solver import *
from robotino_core.solvers.astar_solver import *
from robotino_core.Graph import Graph
from robotino_core.Comm import Comm
from robotino_core.solvers.dmas_solver import *

# Comm
comm = Comm('localhost', 10015, 'localhost', 'matthias', 'matthias', 'kb')
comm.sql_open()

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
new_task = 'pos_16'

# Get tasks in local task list
local_task_list = []

# Get tasks in new local task list
new_local_task_list = local_task_list + [new_task]

# Get start node of robot
start_node = 'pos_1'
start_time = datetime.now()

# Distance function
def distance_function(start_node, end_node):
    best_path, best_slots, best_dist = dmas(graph, start_node, end_node, start_time, comm)
    end_time = best_slots[-1][0] + best_slots[-1][1]
    best_cost = (end_time - start_time).total_seconds()
    return best_path, best_slots, best_dist, best_cost

# Compute cost of current tour
current_solution = tsp(start_node, new_local_task_list, distance_function)
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

print("\nCurent task slots: " + str(current_solution['slots']))

for slots in current_solution['slots']:
    print()
    for slot in slots:
        print(slot)

new_slots = copy(current_solution['slots'])
prev_slot = (current_solution['slots'][0][0][0], timedelta())
for i in range(len(current_solution['slots'])):
    for j in range(len(current_solution['slots'][i])):
        new_slots[i][j] = (prev_slot[0] + prev_slot[1], current_solution['slots'][i][j][1])
        prev_slot = (prev_slot[0] + prev_slot[1], current_solution['slots'][i][j][1])
    
for slots in new_slots:
    print()
    for slot in slots:
        print(slot)
        
print("\nCurent task dists: " + str(current_solution['dists']))
print("Curent task costs: " + str(current_solution['costs']))
print("New task sequence: " + str(new_solution['sequence']))
print("New task paths: " + str(new_solution['paths']))
print("New task slots: " + str(new_solution['slots']))
print("New task dists: " + str(new_solution['dists']))
print("New task costs: " + str(new_solution['costs']))
print("Min sum: " + str(min_sum))
print("Min max: " + str(min_max))
print("Start time: " + str(start_time))
print("End time: " + str(end_time))
print("Duration: " + str(duration))
plt.show()