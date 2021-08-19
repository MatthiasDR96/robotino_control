from robotino_core.solvers.aco_solver import *
from copy import copy, deepcopy


def get_shortest_path(graph, start, dest):

    """

        Implements an ant-colony optimization to find the shortest path using randomness.

        Input:
            - Layout graph
            - Start node name
            - Destination node name
        Output:
            - List of alternative shortest paths - names
            - List of distances - meters
        Default output:
            - []
            - []

    """

    # Get alternative paths 
    path, dist = get_alternative_paths(graph, start, dest, 1)

    return path[0], dist[0]

def get_alternative_paths(graph, start, dest, n):

    """

        Implements an ant-colony optimization to find feasible paths using randomness.

        Input:
            - Layout graph
            - Start node name
            - Destination node name
        Output:
            - List of alternative shortest paths - names
            - List of distances - meters
        Default output:
            - []
            - []

    """

    # Assertions
    assert isinstance(start, str)
    assert isinstance(dest, str)
    assert isinstance(n, int)

    # Overwrite Ant functions
    Ant.can_move = can_move
    Ant.remaining_moves = remaining_moves

    # Take deepcopy of graph
    graph = deepcopy(graph)

    # Init solution
    feasible_paths = []
    feasible_distances = []

    # Init params
    num_iter = 0

    # Reset all edge pheromones
    graph.reset_edge_pheromone(t0)

    # Create ant colony
    colony = create_colony(graph, start, [dest])

    # Loop
    global_best = None
    while len(feasible_paths) < n and num_iter < iterations:

        # Perform one aco-optimization
        local_best = aco(colony)

        # If solution exists
        if local_best:

            # Save local best solution
            if local_best.visited not in feasible_paths:
                feasible_paths.append(local_best.visited)
                feasible_distances.append(local_best.traveled_cost)

            # Save global best ant
            if global_best is None or local_best < global_best:
                global_best = copy(local_best)

        # Raise pheromone of global best ant track
        if global_best:
            trace_elite(global_best)

        # Increase iter
        num_iter += 1

    # Sort for distance
    sorted_indices = [i[0] for i in sorted(enumerate(feasible_distances), key=lambda x:x[1])]
    feasible_paths = [feasible_paths[i] for i in sorted_indices]
    feasible_distances = [feasible_distances[i] for i in sorted_indices]
    
    return feasible_paths, feasible_distances


def can_move(self):
        """Returns false if all nodes in `to_visit` are visited or if all neighbors are already visited"""

        # Check if all nodes to visit are visited
        set1 = set(self.to_visit)
        set2 = set(self.visited)
        visited_all = set1.issubset(set2)

        # Check if there are some neighbors not yet visited
        set3 = set(self.graph.nodes[self.visited[-1]].neighbors)
        set4 = set(self.visited)
        all_neighbors_visited = set3.issubset(set4)

        return not visited_all and not all_neighbors_visited

def remaining_moves(self):
        """Return the moves that can be made from this node."""
        current_node = self.visited[-1]
        neighbors = self.graph.nodes[current_node].neighbors
        return [x for x in neighbors if x not in self.visited]
