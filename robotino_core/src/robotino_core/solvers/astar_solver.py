import math
from random import random
from copy import deepcopy

def get_alternative_paths(graph, start, dest, n):

    """

        Implements the multi - astar shortest path solver using the Penalty approach.

        Input:
            - Layout graph
            - Start node name
            - Destination node name
            - Number of alternative paths
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

    # Take deepcopy of graph
    graph = deepcopy(graph)

    # Init solution
    feasible_paths = []
    feasible_distances = []

    # Init params
    num_fails = 0
    max_fails = 100
    penalty = 1000
    alpha = random()

    # Loop
    while len(feasible_paths) < n and num_fails < max_fails:

        # Astar single-path algorithm
        path, dist = get_shortest_path(graph, start, dest)

        # Add path if not yet found
        if path in feasible_paths:
            num_fails += 1
        else:
            feasible_paths.append(path)
            feasible_distances.append(dist)
            num_fails = 0
        
        # Penalize path
        for i in range(len(path)-1):
            beta = random()
            if beta < alpha:
                graph.edges[path[i], path[i+1]].pheromone += penalty

    # Sort for distance
    sorted_indices = [i[0] for i in sorted(enumerate(feasible_distances), key=lambda x:x[1])]
    feasible_paths = [feasible_paths[i] for i in sorted_indices]
    feasible_distances = [feasible_distances[i] for i in sorted_indices]

    return feasible_paths, feasible_distances

def get_shortest_path(graph, start_node, end_node):

    """

        Implements the single - astar shortest path solver.

        Input:
            - Layout graph
            - Start node name
            - End node name
        Output:
            - Shortest path - names
            - Distance - meters
        Default output:
            - None
            - None
    """

    # Assertions
    assert isinstance(start_node, str)
    assert isinstance(end_node, str)

    # Take deepcopy of graph
    graph = deepcopy(graph)

    # Get nodes from node names
    start_node = graph.nodes[start_node]
    end_node = graph.nodes[end_node]

    # Init open and closed set
    openset = set()
    closedset = set()

    # Add start node to open set
    current = start_node
    openset.add(current)

    # Loop until openset is empty
    while openset:

        # Take node with least cost as next node
        current = min(openset, key=lambda o: o.g + o.h)

        # End criterium
        if current.name == end_node.name:
            path = []
            distance = 0
            while current.parent:
                path.append(current.name)
                distance += graph.edges[current.parent.name, current.name].length
                current = current.parent
            path.append(current.name)
            return path[::-1], distance

        # Move to next node
        openset.remove(current)
        closedset.add(current)

        # Explore neighbors
        for neighbor_name in current.neighbors:

            # Get neighbor node
            neighbor = graph.nodes[neighbor_name]

            # If neighbors already visited, skip
            if neighbor in closedset:
                continue 

            # Compute new cost to neighbor
            new_g = current.g + graph.edges[current.name, neighbor.name].length + graph.edges[current.name, neighbor.name].pheromone # math.sqrt(math.pow(self.pos[0] - node.pos[0], 2) + math.pow(self.pos[1] - node.pos[1], 2))
        
            # If neighbor in open set, update cost if lower than before
            if neighbor in openset :
                if neighbor.g > new_g:
                    neighbor.g = new_g
                    neighbor.parent = current

            # If neighbor not in open set, update cost and add to openset
            else:
                neighbor.g = new_g
                neighbor.h = heuristic(neighbor, end_node)
                neighbor.parent = current
                openset.add(neighbor)

    return None, None


def heuristic(node_a, node_b):
    return math.sqrt(math.pow(node_a.pos[0] - node_b.pos[0], 2) + math.pow(node_a.pos[1] - node_b.pos[1], 2))
