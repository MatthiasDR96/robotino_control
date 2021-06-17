import os
import yaml
from robotino_core.Graph import Graph
import matplotlib.pyplot as plt

# Read params
print(os.getcwd())
this_dir = os.path.dirname(os.path.dirname(__file__))
data_path = os.path.join(this_dir, "src/robotino_core/params", "setup2.yaml")
with open(data_path, 'r') as file:
	params = yaml.load(file, Loader=yaml.FullLoader)

# Create graph
graph = Graph()
node_neighbors = params['node_neighbors']
node_locations = params['node_locations']
graph.create_nodes(list(node_locations.values()), list(node_locations.keys()))
graph.create_edges(list(node_neighbors.keys()), list(node_neighbors.values()))

graph.plot()
plt.show()

# reading png image file
im = plt.imread('src/robotino_core/params/map.pgm')
  
# show image
fig, ax = plt.subplots()
ax.imshow(im, extent=[0, im.shape[0], 0, im.shape[0]])
for i in range(len(node_locations.values())):
	pos = list(node_locations.values())[i]
	u = pos[0]/0.05 + 10/0.05
	v = pos[1]/0.05 + 10/0.05
	ax.plot(u, v, 'r.')
plt.show()