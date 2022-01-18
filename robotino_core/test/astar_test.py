import matplotlib.pyplot as plt
import robotino_core.solvers.astar_solver as asm
from robotino_core.Comm import Comm

# Init database communication
comm = Comm('127.0.0.1', 10015, '127.0.0.1', 'root', 'abc123', 'kb')

# Get graph
graph = comm.sql_select_graph()
print(graph)

# Start location
start = 'pos_1'

# Destination(s)
dest = 'pos_14'

# Find single shortest path
path, dist = asm.get_shortest_path(graph, start, dest)

# Print results
print("\nSingle shortest path:")
print(path)
print(dist)

# Find multi shortest path
path, dist = asm.get_alternative_paths(graph, start, dest, 10)

# Print results
print("\nMulti shortest path:")
for i in range(len(path)):
    print(path[i])
    print(dist[i])

# Plot graph
graph.plot()
plt.show()