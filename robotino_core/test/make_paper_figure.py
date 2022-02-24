import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

from robotino_core.Graph import Graph

# Define layout
node_locations = [(10, 30), (10, 50), (10, 70), (30, 10), (30, 30), (30, 50), (30, 70), (30, 90), (60, 10),
                  (60, 30), (60, 50), (60, 70), (60, 90), (90, 10), (90, 50), (90, 90)]
node_names = ["pos_1", "pos_2", "pos_3", "pos_4", "pos_5", "pos_6", "pos_7", "pos_8", "pos_9", "pos_10",
              "pos_11", "pos_12", "pos_13", "pos_14", "pos_15", "pos_16"]
depot_names = ["pos_1", "pos_2", "pos_3"]
task_names = ["pos_4", "pos_5", "pos_6", "pos_7", "pos_8", "pos_9", "pos_10", "pos_11", "pos_12", "pos_13",
              "pos_14", "pos_15", "pos_16"]
node_neighbors = [["pos_5"], ["pos_6"], ["pos_7"], ["pos_5", "pos_9"], ["pos_1", "pos_4", "pos_6"],
                  ["pos_2", "pos_5", "pos_7", "pos_10", "pos_11", "pos_12"],
                  ["pos_3", "pos_6", "pos_8"], ["pos_7", "pos_13"], ["pos_4", "pos_10", "pos_14"],
                  ["pos_6", "pos_9", "pos_11"],
                  ["pos_6", "pos_10", "pos_12", "pos_15"], ["pos_6", "pos_11", "pos_13"],
                  ["pos_8", "pos_12", "pos_16"], ["pos_15", "pos_9"], ["pos_11", "pos_14", "pos_16"],
                  ["pos_13", "pos_15"]]

# Create layout
graph = Graph()
graph.create_nodes(node_locations, node_names)
graph.create_edges(node_names, node_neighbors)

### Make figure 1 ###

# Make figure
plt.figure(2)
fig, ax = plt.subplots(1, 1)
ax.set_title('Layout')
ax.set_xlabel('x-coordinate (m)')
ax.set_ylabel('y-coordinate (m)')

for node in graph.nodes.values():
    ax.plot(node.pos[0], node.pos[1], 'b.', ms=6)
    ax.text(node.pos[0] + 1.5, node.pos[1] + 1.5, node.name)
for edge in graph.edges.values():
    arrow_x = (edge.start_node.pos[0] + edge.end_node.pos[0]) / 2
    arrow_y = (edge.start_node.pos[1] + edge.end_node.pos[1]) / 2
    delta_x = (edge.end_node.pos[0] - edge.start_node.pos[0]) * 0.01
    delta_y = (edge.end_node.pos[1] - edge.start_node.pos[1]) * 0.01
    ax.plot([edge.start_node.pos[0], edge.end_node.pos[0]], [edge.start_node.pos[1], edge.end_node.pos[1]],
            'b-', lw=0.5)
    ax.arrow(arrow_x, arrow_y, delta_x, delta_y, length_includes_head=True, head_width=2, head_length=2)

# Green path and nodes
path = ['pos_3', 'pos_7', 'pos_8', 'pos_13', 'pos_16']
nodes = ['pos_8', 'pos_16']
ax.plot(graph.nodes[path[0]].pos[0], graph.nodes[path[0]].pos[1], 'g^', ms=8)
for i in range(len(path) - 1):
    ax.plot([graph.nodes[path[i]].pos[0], graph.nodes[path[i + 1]].pos[0]],
            [graph.nodes[path[i]].pos[1], graph.nodes[path[i + 1]].pos[1]], 'g-', lw=2.5)
for i in range(len(nodes)):
    ax.plot(graph.nodes[nodes[i]].pos[0], graph.nodes[nodes[i]].pos[1], 'gs', ms=10)

# Yellow path
path = ['pos_2', 'pos_6', 'pos_11']
nodes = ['pos_11']
ax.plot(graph.nodes[path[0]].pos[0], graph.nodes[path[0]].pos[1], 'y^', ms=8)
for i in range(len(path) - 1):
    ax.plot([graph.nodes[path[i]].pos[0], graph.nodes[path[i + 1]].pos[0]],
            [graph.nodes[path[i]].pos[1], graph.nodes[path[i + 1]].pos[1]], 'y-', lw=2.5)
for i in range(len(nodes)):
    ax.plot(graph.nodes[nodes[i]].pos[0], graph.nodes[nodes[i]].pos[1], 'ys', ms=10)

# Blue path
path = ['pos_1', 'pos_5', 'pos_4']
nodes = ['pos_4']
ax.plot(graph.nodes[path[0]].pos[0], graph.nodes[path[0]].pos[1], 'r^', ms=8)
for i in range(len(path) - 1):
    ax.plot([graph.nodes[path[i]].pos[0], graph.nodes[path[i + 1]].pos[0]],
            [graph.nodes[path[i]].pos[1], graph.nodes[path[i + 1]].pos[1]], 'r-', lw=2.5)
for i in range(len(nodes)):
    ax.plot(graph.nodes[nodes[i]].pos[0], graph.nodes[nodes[i]].pos[1], 'rs', ms=10)

# Save
plt.savefig('layout.png', dpi=400)

### Make figure 2 ###

# Make figure
plt.figure(2)
fig, ax = plt.subplots(1, 1)
ax.set_title('Layout')
ax.set_xlabel('x-coordinate (m)')
ax.set_ylabel('y-coordinate (m)')

for node in graph.nodes.values():
    ax.plot(node.pos[0], node.pos[1], 'b.', ms=6)
    ax.text(node.pos[0] + 1.5, node.pos[1] + 1.5, node.name)
for edge in graph.edges.values():
    arrow_x = (edge.start_node.pos[0] + edge.end_node.pos[0]) / 2
    arrow_y = (edge.start_node.pos[1] + edge.end_node.pos[1]) / 2
    delta_x = (edge.end_node.pos[0] - edge.start_node.pos[0]) * 0.01
    delta_y = (edge.end_node.pos[1] - edge.start_node.pos[1]) * 0.01
    ax.plot([edge.start_node.pos[0], edge.end_node.pos[0]], [edge.start_node.pos[1], edge.end_node.pos[1]],
            'b-', lw=0.5)
    ax.arrow(arrow_x, arrow_y, delta_x, delta_y, length_includes_head=True, head_width=2, head_length=2)

# Black path and nodes
path = ['pos_3', 'pos_7', 'pos_8', 'pos_13', 'pos_16']
nodes = ['pos_16']
ax.plot(graph.nodes[path[0]].pos[0], graph.nodes[path[0]].pos[1], 'g^', ms=8)
for i in range(len(path) - 1):
    ax.plot([graph.nodes[path[i]].pos[0], graph.nodes[path[i + 1]].pos[0]],
            [graph.nodes[path[i]].pos[1], graph.nodes[path[i + 1]].pos[1]], 'g-', lw=2.5)
for i in range(len(nodes)):
    ax.plot(graph.nodes[nodes[i]].pos[0], graph.nodes[nodes[i]].pos[1], 'gs', ms=10)

# Save and plot
plt.savefig('layout.png', dpi=400)
plt.show()