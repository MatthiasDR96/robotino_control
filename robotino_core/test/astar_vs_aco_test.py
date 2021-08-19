from datetime import datetime
import matplotlib.pyplot as plt
import robotino_core.solvers.astar_solver as astar
import robotino_core.solvers.feasibility_ant_solver as aco
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

# Number of solutions
n = 5

# Find single shortest path
start_astar = datetime.now()
path, dist = astar.get_shortest_path(graph, start, dest)
stop_astar = datetime.now()
diff_astar = (stop_astar - start_astar).total_seconds()

# Print results
print("\nAstar path:   took " + str(diff_astar) + " seconds")
print(path)

# Find multi shortest path
start_astar = datetime.now()
path, dist = astar.get_alternative_paths(graph, start, dest, n)
stop_astar = datetime.now()
diff_astar = (stop_astar - start_astar).total_seconds()

# Print results
print("\nAstar paths:   took " + str(diff_astar) + " seconds")
for i in range(len(path)):
    print(path[i])

# Find single shortest path
start_aco = datetime.now()
path, dist = aco.get_shortest_path(graph, start, dest)
stop_aco = datetime.now()
diff_aco = (stop_aco - start_aco).total_seconds()

# Print results
print("\nAco path:   took " + str(diff_aco) + " seconds")
print(path)

# Find multi shortest path
start_aco = datetime.now()
path, dist = aco.get_alternative_paths(graph, start, dest, n)
stop_aco = datetime.now()
diff_aco = (stop_aco - start_aco).total_seconds()

# Print results
print("\nAco paths:   took " + str(diff_aco) + " seconds")
for i in range(len(path)):
    print(path[i])