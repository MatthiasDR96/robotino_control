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

# Create rm-agent

# Get tasks in local task list
task_sequence = ['pos_5', 'pos_8']

# Get start node of robot
start_node = 'pos_1'
init_resources = 100

# Calculate resources at each node without charging
resources_at_nodes = self.calculate_resources_at_nodes(task_sequence, start_node, init_resources, None, 0)

# Insert charging station if needed
if resources_at_nodes[-1] <= self.agv.battery_threshold:

    # Compute optimal insertion point, charging station, and charging percent
    charging_station, insertion_index, charging_percent = self.optimize_brute_force(task_sequence, start_node, init_resources)