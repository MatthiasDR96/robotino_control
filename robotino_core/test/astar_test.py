import matplotlib.pyplot as plt
import robotino_core.solvers.astar_solver_multi as asm
from robotino_core.Comm import Comm

# Init database communication
comm = Comm('localhost', 10015, 'localhost', 'matthias', 'matthias', 'kb')
comm.sql_open()

# Get graph
graph = comm.get_graph()

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