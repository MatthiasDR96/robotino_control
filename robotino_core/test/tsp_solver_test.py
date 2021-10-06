import numpy as np
import matplotlib.pyplot as plt
from robotino_core.solvers.tsp_solver import *
from robotino_core.solvers.astar_solver import *
from robotino_core.Comm import Comm

# Init database communication
comm = Comm('localhost', 10015, 'localhost', 'matthias', 'matthias', 'kb')
comm.sql_open()

# Get graph
graph = comm.get_graph()

def dist_astar(a, b):
		return get_shortest_path(graph, a, b)

if __name__ == "__main__":

    # Start node
    start_node = 'pos_2'

    # Nodes to visit
    nodes_to_visit = ['pos_5', 'pos_9']

    # Solve
    solution = tsp(start_node, nodes_to_visit, dist_astar)
    print("\nShortest task sequence names: " + str(solution['sequence']))
    print("\nShortest task paths names: " + str(solution['paths']))
    print("\nShortest task dists: " + str(solution['dists']))
    print("Distance (m): " + str(sum(solution['dists'])))

    # Plot
    ax = plt.subplot()
    graph.plot(ax)
    plt.show()