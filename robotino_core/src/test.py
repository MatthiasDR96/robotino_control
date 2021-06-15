import os
import yaml
from robotino_core.Graph import Graph

# Read params
this_dir = os.path.dirname(os.path.dirname(__file__))
data_path = os.path.join(this_dir, "src/robotino_core/params", "test.yaml")
with open(data_path, 'r') as file:
	params = yaml.load(file, Loader=yaml.FullLoader)

# Create graph
graph = Graph()
node_names = params['node_names']
node_locations = params['node_locations']
graph.create_nodes(node_locations, list(node_names.keys()))
graph.create_edges(list(node_names.keys()), list(node_names.values()))

graph.print_nodes()
graph.print_edges()