
from robotino_core.solvers.aco_solver import aco_solve
from robotino_core.solvers.aco_solver import Ant
from robotino_core.solvers.astar_solver import *


def tsp(start, dest, distance_function):

    """

        Implements an ant-colony optimization to solve the TSP problem. Routes between nodes are calculated using astar.

        Input:
            - Start node name
            - Destination nodes name
            - Distance function
        Output:
            - TSP solution - dict
        Default output:
            - None
    """

    # Assertions
    assert isinstance(start, str)
    assert isinstance(dest, list)

    # Create TSP world
    world = World([start] + dest, distance_function)

    # Overwrite Ant functions
    Ant.can_move = can_move
    Ant.remaining_moves = remaining_moves

    # Solve aco
    global_best = aco_solve(world, start, dest)

    # Output
    if global_best:
        solution = {}
        solution['sequence'] = global_best.sequence[1:]
        solution['paths'] = [edge.nodes_in_between for edge in global_best.edges]
        solution['slots'] = [edge.slots_in_between for edge in global_best.edges]
        solution['dists'] = [edge.dist for edge in global_best.edges]
        solution['costs'] = [edge.cost for edge in global_best.edges]
    else:
        solution = None

    return solution

class World:

    def __init__(self, nodes, distance_function):
        self.distance_function = distance_function
        self.nodes = nodes
        self.edges = self.create_edges()

    def create_edges(self):
        edges = {}
        for m in self.nodes:
            for n in self.nodes:
                if m != n:
                    nodes_in_between, slots_in_between, dist, cost = self.distance_function(m, n)
                    edges[m, n] = Edge(m, n, nodes_in_between, slots_in_between, dist, cost)
        return edges

    def reset_edge_pheromone(self, level=0.01):
        for edge in self.edges.values():
            edge.pheromone = level

class Edge:

    def __init__(self, start_node, end_node, nodes_in_between, slots_in_between, dist, cost, pheromone=0.1):
        self.start_node = start_node
        self.end_node = end_node
        self.nodes_in_between = nodes_in_between
        self.slots_in_between = slots_in_between
        self.dist = dist
        self.cost = cost
        self.pheromone = pheromone

    def __str__(self):
        return str(self.start_node) + " => " + str(self.end_node)

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.__dict__ == other.__dict__
        return False

def can_move(self):
        """Returns false if all nodes in `to_visit` are visited or if all neighbors are already visited"""

        # Check if all nodes to visit are visited
        set1 = set(self.to_visit)
        set2 = set(self.visited)
        visited_all = set1.issubset(set2)

        return not visited_all

def remaining_moves(self):
        """Return the moves that can be made from this node."""
        return self.unvisited


