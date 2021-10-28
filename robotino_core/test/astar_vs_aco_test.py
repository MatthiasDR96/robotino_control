from datetime import datetime
import matplotlib.pyplot as plt
import sys
sys.setrecursionlimit(10000)
import robotino_core.solvers.astar_solver as astar
import robotino_core.solvers.feasibility_ant_solver as aco
from robotino_core.Comm import Comm

# Init database communication
comm = Comm('localhost', 10015, 'localhost', 'matthias', 'matthias', 'kb')
comm.sql_open()

# Get graph
graph = comm.sql_get_graph()

# Start location
start = 'pos_1'

# Destination(s)
dest = 'pos_52'

# Number of solutions
n = 5

# Find multi shortest path
start_astar = datetime.now()
global_best, local_best = astar.get_alternative_paths(graph, start, dest, n)
stop_astar = datetime.now()
diff_astar = (stop_astar - start_astar).total_seconds()

# Print results
print("\nAstar paths:   took " + str(diff_astar) + " seconds")
for i in range(len(local_best)):
    print(local_best[i])

# Find multi shortest path
start_aco = datetime.now()
global_best, local_best = aco.get_alternative_paths(graph, start, [dest], n)
stop_aco = datetime.now()
diff_aco = (stop_aco - start_aco).total_seconds()

# Print results
print("\nAco paths:   took " + str(diff_aco) + " seconds")
for i in range(len(local_best)):
    print(local_best[i])