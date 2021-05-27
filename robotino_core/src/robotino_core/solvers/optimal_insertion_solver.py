import bisect
import itertools
import random
from copy import copy

from src.solvers.astar_solver import *

alpha = 1  # relative importance of pheromone (default=1)
beta = 3  # relative importance of distance (default=3)
rho = 0.8  # percent evaporation of pheromone (0..1, default=0.8)
q = 1  # total pheromone deposited by each :class:`Ant` after each iteration is complete (>0, default=1)
t0 = 0.01  # initial pheromone level along each :class:`Edge` of the :class:`World` (>0, default=0.01)
iterations = 100  # number of iterations to perform (default=100)
ant_count = 10  # how many :class:`Ant`\s will be used (default=10)
elite = 0.5  # multiplier of the pheromone deposited by the elite :class:`Ant` (default=0.5)


def optimal_insertion_solve(graph, start_node, nodes_to_visit):
    """
        Input:
            - Total layout graph
            - Start node name
            - Node to visit names
        Output:
            - Shortest task sequence (without starting node)
            - Shortest tour containing all tasks
            - Distance in meters
        Default output:
            - [start]
            - [start]
            - 0
    """

    # Tak deepcopy of graph
    graph = deepcopy(graph)

    # If no nodes to visit, return start position
    if len(nodes_to_visit) == 0:
        global_best_task_sequence = [start_node]
        global_best_tour = [start_node]
        tour_cost = 0

    else:

        # Create problem graph
        graph = World(graph, [start_node] + nodes_to_visit)

        # Reset all edge pheromones
        graph.reset_pheromone(t0)

        # Init
        global_best = None

        # Create ant colony
        colony = create_colony(graph, start_node, nodes_to_visit)

        # Loop through iterations
        for i in range(iterations):

            # Perform one aco-optimization
            local_best = aco(colony)

            # save global best ant
            if global_best is None or local_best < global_best:
                global_best = copy(local_best)

            # Raise pheromone of global best ant track
            trace_elite(global_best)

        # Save solution
        global_best_task_sequence = global_best.tour
        global_best_tour = global_best.complete_tour
        tour_cost = global_best.travelled_time

    return global_best_task_sequence[1:], global_best_tour, tour_cost


def create_colony(graph, start, to_visit):
    """Create a set of :class:`Ant` and initialize them to the given graph"""
    return [Ant(graph, start, to_visit).initialize() for _ in range(ant_count)]


def aco(colony):
    """Return the best solution by performing the ACO meta-heuristic."""

    # Reset colony
    reset_colony(colony)

    # Let ants propagate till destination reached
    find_solutions(colony)

    # Perform a global evaporation
    global_update(colony)

    # Collect valid ant solutions
    valid_colony = [ant for ant in colony if set(ant.to_visit).issubset(set(ant.visited))]

    # Return local best solution
    if not len(valid_colony) == 0:
        best = sorted(valid_colony)[0]
    else:
        best = None
    return best


def reset_colony(colony):
    """Reset the *colony* of :class:`Ant` such that each :class:`Ant` is ready to find a new solution."""
    for ant in colony:
        ant.initialize()


def find_solutions(ants):
    """Let each :class:`Ant` find a solution. Makes each :class:`Ant` move until each can no longer move."""
    ants_done = 0
    while ants_done < len(ants):
        ants_done = 0
        for ant in ants:
            if ant.can_move():
                edge = ant.move()
                local_update(edge)
            else:
                ants_done += 1


def local_update(edge):
    """Evaporate some of the pheromone on the given *edge*."""
    edge.pheromone = max(t0, edge.pheromone * rho)


def global_update(ants):
    """Update the amount of pheromone on each edge according to the fitness of solutions that use it."""
    ants = sorted(ants)[:len(ants) // 2]
    for a in ants:
        p = q / a.travelled_time if not a.travelled_time == 0 else t0
        for edge in a.path:
            edge.pheromone = max(t0, (1 - rho) * edge.pheromone + p)


def trace_elite(ant):
    """Deposit pheromone along the path of a particular ant."""
    if elite:
        p = elite * q / ant.travelled_time if not ant.travelled_time == 0 else t0
        for edge in ant.path:
            edge.pheromone += p


class World:

    def __init__(self, graph, nodes):
        self.graph = graph
        self.nodes = nodes
        self.edges = self.create_edges()

    def create_edges(self):
        edges = {}
        for m in self.nodes:
            for n in self.nodes:
                if m != n:
                    nodes_in_between, length = find_shortest_path(self.graph, m, n)
                    edge = Edge(m, n, nodes_in_between=nodes_in_between, length=length)
                    edges[m, n] = edge
        return edges

    def reset_pheromone(self, level=0.01):
        for edge in self.edges.values():
            edge.pheromone = level


class Edge:

    def __init__(self, start, end, nodes_in_between=None, length=None, pheromone=None):
        self.start = start
        self.end = end
        self.nodes_in_between = nodes_in_between
        self.length = 1 if length is None else length
        self.pheromone = 0.1 if pheromone is None else pheromone

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.__dict__ == other.__dict__
        return False


class Ant:

    def __init__(self, graph, start, to_visit, alpha=1, beta=3):

        # Performance attributes
        self.alpha = alpha  # Importance factor of the pheromones
        self.beta = beta  # Importance factor of the distance

        # Task related attributes
        self.graph = graph  # Graph in which the ant can move
        self.start = start  # Node from which the ant needs to start
        self.to_visit = to_visit  # Nodes the ant needs to visit

        # Updated attributes
        self.travelled_time = 0  # Total time traveled
        self.traveled = []  # Edges traveled
        self.visited = []
        self.unvisited = []

        # Tour
        self.tour_ = []

    def __eq__(self, other):
        """Returns true if distances are equal"""
        return self.travelled_time == other.travelled_time

    def __lt__(self, other):
        """Returns true if other distance is larger"""
        return self.travelled_time < other.travelled_time

    def initialize(self):
        self.travelled_time = 0
        self.traveled = []
        self.visited = [self.start]
        self.tour_ = [self.start]
        self.unvisited = [n for n in self.graph.nodes if n != self.start]
        return self

    @property
    def complete_tour(self):
        """Nodes traveled by the :class:`Ant` in order."""
        return [node for node in self.tour_]

    @property
    def tour(self):
        """Nodes traveled by the :class:`Ant` in order."""
        return [node for node in self.visited]

    @property
    def path(self):
        """Edges traveled by the :class:`Ant` in order."""
        return [edge for edge in self.traveled]

    def can_move(self):
        """Returns false if all nodes in `to_visit` are visited or if all neighbors are already visited"""

        # Check if all nodes to visit are visited
        set1 = set(self.to_visit)
        set2 = set(self.visited)
        visited_all = set1.issubset(set2)
        return not visited_all

    def move(self):
        """Choose, make, and return a move from the remaining moves."""
        remaining = self.remaining_moves()
        choice = self.choose_move(remaining)
        return self.make_move(choice)

    def remaining_moves(self):
        """Return the moves that can be made from this node."""
        return self.unvisited

    def choose_move(self, choices):
        """Choose a move from all possible moves."""
        if len(choices) == 0:
            return None
        if len(choices) == 1:
            return choices[0]

        # Find the weight of the edges that take us to each of the choices.
        weights = []
        for move in choices:
            current_node = self.visited[-1]
            if not move == current_node:
                edge = self.graph.edges[current_node, move]
                weights.append(self.weigh(edge))

        # Choose one of them using a weighted probability.
        total = sum(weights)
        cumdist = list(itertools.accumulate(weights)) + [total]
        return choices[bisect.bisect(cumdist, random.random() * total)]

    def make_move(self, dest):
        """Move to the *dest* node and return the edge traveled."""
        current_node = self.visited[-1]

        self.visited.append(dest)
        self.unvisited.remove(dest)

        edge = self.graph.edges[current_node, dest]
        self.traveled.append(edge)
        self.tour_ += edge.nodes_in_between[1:]
        self.travelled_time += edge.length
        return edge

    def weigh(self, edge):
        """Calculate the weight of the given *edge*."""
        pre = 1 / (edge.length or 1)
        post = edge.pheromone
        return post ** self.alpha * pre ** self.beta
