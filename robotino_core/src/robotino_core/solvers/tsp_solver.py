import bisect
import itertools
import random
from copy import copy

alpha = 1  # relative importance of pheromone (default=1)
beta = 3  # relative importance of distance (default=3)
rho = 0.8  # percent evaporation of pheromone (0..1, default=0.8)
q = 1  # total pheromone deposited by each :class:`Ant` after each iteration is complete (>0, default=1)
t0 = 0.01  # initial pheromone level along each :class:`Edge` of the :class:`World` (>0, default=0.01)
iterations = 50  # number of iterations to perform (default=100)
ant_count = 5  # how many :class:`Ant`\s will be used (default=10)
elite = 0.5  # multiplier of the pheromone deposited by the elite :class:`Ant` (default=0.5)


def tsp(start_node, nodes_to_visit, distance_function):

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
    assert isinstance(nodes_to_visit, list)

    # Create aco world
    world = World([start_node] + nodes_to_visit, distance_function)

    # Solve aco
    global_best, local_best = aco_solve(world, start_node, nodes_to_visit, 1000)

    # Output
    if global_best:
        solution = dict()
        solution['sequence'] = global_best.visited[1:]
        solution['paths'] = [edge.nodes_in_between[1:] for edge in global_best.edges]
        solution['dists'] = [edge.length for edge in global_best.edges]
    else:
        solution = None

    return solution


def aco_solve(world, start_node, nodes_to_visit, n):
    """
        Implements the ant colony optimization algorithm that finds an optimal path in the world through all nodes
        Input:
            - Layout graph
            - Start node name
            - Destination node(s) name
            - Number of alternative paths
        Output:
            - List of alternative shortest paths - names
            - List of distances - meters
        Default output:
            - []
            - []

    """

    # Assertions
    assert isinstance(nodes_to_visit, list)

    # Init solution
    local_best_sol = []
    global_best_sol = None

    # Init params
    num_iter = 0

    # Reset all edge pheromones
    world.reset_edge_pheromone(t0)

    # Create ant colony
    colony = create_colony(world, start_node, nodes_to_visit)

    # Loop
    while len(local_best_sol) < n and num_iter < iterations:

        # Perform one aco-optimization
        local_best = aco(colony)

        # If solution exists
        if local_best:

            # Save local best solution
            if local_best not in local_best_sol:
                local_best_sol.append(copy(local_best))

            # Save global best ant
            if global_best_sol is None or local_best < global_best_sol:
                global_best_sol = copy(local_best)

        # Raise pheromone of global best ant track
        if global_best_sol:
            trace_elite(global_best_sol)

        # Increase iter
        num_iter += 1

    return global_best_sol, local_best_sol


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
        p = q / a.dist if not a.dist == 0 else t0
        for edge in a.edges:
            edge.pheromone = max(t0, (1 - rho) * edge.pheromone + p)


def trace_elite(ant):
    """Deposit pheromone along the path of a particular ant."""
    if elite:
        p = elite * q / ant.dist if not ant.dist == 0 else t0
        for edge in ant.edges:
            edge.pheromone += p


class World:

    def __init__(self, nodes, distance_function):
        self.nodes = nodes
        self.distance_function = distance_function
        self.edges = self.create_edges()

    def create_edges(self):
        edges = {}
        for m in self.nodes:
            for n in self.nodes:
                if m != n:
                    nodes_in_between, dist = self.distance_function(m, n)
                    edges[m, n] = Edge(m, n, nodes_in_between, dist)
        return edges

    def reset_edge_pheromone(self, level=0.01):
        for edge in self.edges.values():
            edge.pheromone = level


class Edge:

    def __init__(self, start_node, end_node, nodes_in_between, length, pheromone=0.1):
        self.start_node = start_node
        self.end_node = end_node
        self.nodes_in_between = nodes_in_between
        self.length = length
        self.pheromone = pheromone

    def __str__(self):
        return str(self.start_node) + " => " + str(self.end_node)

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
        self.dist = 0.0
        self.edges = []  # Edges traveled
        self.visited = []  # Nodes visited
        self.unvisited = []  # Nodes unvisited

    def __eq__(self, other):
        """Returns true if distances are equal"""
        return self.dist == other.dist

    def __lt__(self, other):
        """Returns true if other distance is larger"""
        return self.dist < other.dist

    def initialize(self):
        self.dist = 0
        self.edges = []
        self.visited = [self.start]
        self.unvisited = [n for n in self.graph.nodes if n != self.start]
        return self

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

    def move(self):
        """Choose, make, and return a move from the remaining moves."""
        remaining = self.remaining_moves()
        choice = self.choose_move(remaining)
        return self.make_move(choice)

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
        self.edges.append(edge)
        self.dist += edge.length
        return edge

    def weigh(self, edge):
        """Calculate the weight of the given *edge*."""
        pre = 1 / (edge.length or 1)
        post = edge.pheromone
        return post ** self.alpha * pre ** self.beta