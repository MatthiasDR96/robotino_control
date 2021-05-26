import math
import time
import mysql.connector
import configparser
import ast
import threading

from robotino_core.Comm import Comm
from robotino_core.Graph import Graph
from robotino_core.datatypes.Task import Task
from robotino_core.datatypes.Robot import Robot
from robotino_core.agv.AGV_Action import Action
from robotino_core.agv.AGV_Routing import Routing
from robotino_core.agv.AGV_TaskAllocation import TaskAllocation
from robotino_core.agv.AGV_ResourceManagement import ResourceManagement
from robotino_core.solvers.astar_solver import *


class AGV:
    """
        A class containing the intelligence of the agv agent
    """

    def __init__(self, ip, port, loc):

        # Setup
        self.setup_file_path = 'setup.ini'
        self.setup = configparser.ConfigParser()
        self.setup.read(self.setup_file_path)

        # Communication attributes
        self.ip = ip
        self.port = port
        self.comm = Comm(self.ip, self.port)

        # Connect to database
        self.mydb = mysql.connector.connect(
            host="localhost",
            user="matthias",
            password="matthias",
            database='kb'
        )

        # Agv attributes
        self.id = None
        self.speed = float(self.setup['GENERAL']['robot_speed'])  # Robot speed
        self.task_execution_time = float(self.setup['GENERAL']['task_execution_time'])  # Execution time of tasks
        self.battery_threshold = float(self.setup['GENERAL']['battery_threshold'])
        self.collision_threshold = float(self.setup['GENERAL']['collision_threshold'])
        self.max_charging_time = float(self.setup['GENERAL']['max_charging_time'])
        self.max_tasks_in_task_list = float(self.setup['GENERAL']['max_tasks_in_task_list'])
        self.initial_resources = float(self.setup['GENERAL']['initial_resources'])
        self.depot_locations = ast.literal_eval(self.setup['LAYOUT']['depot_locations'])  # List of depot locations
        self.dmas_update_rate = 2

        # Create graph
        self.node_locations = ast.literal_eval(self.setup['LAYOUT']['node_locations'])  # List of locations of nodes
        self.node_neighbors = ast.literal_eval(self.setup['LAYOUT']['node_neighbors'])  # List of edges between nodes
        self.node_names = ast.literal_eval(self.setup['LAYOUT']['node_names'])  # List of node names
        self.graph = Graph()
        self.graph.create_nodes(self.node_locations, self.node_names)
        self.graph.create_edges(self.node_names, self.node_neighbors)

        # Control typesr
        self.charging_approach = False
        self.routing_approach = False

        # Updated attributes
        self.x_loc = loc[0]
        self.y_loc = loc[1]
        self.theta = 0.0
        self.node = self.search_closest_node(loc)
        self.status = 'IDLE'
        self.battery_status = 100.0
        self.travelled_time = 0.0
        self.charged_time = 0.0
        self.congestions = 0
        self.task_executing = Task([-1, None])
        self.path = []
        self.total_path = []
        
        # Dmas attribute
        self.slots = []
        self.reserved_paths = {}
        self.reserved_slots = {}
        
        # Add robot to database
        robot = Robot([None, self.ip, self.port, self.x_loc, self.y_loc, self.theta, self.node,
                self.status, self.battery_status, self.travelled_time, self.charged_time,
                self.congestions, self.task_executing.id, str(self.path), str(self.total_path)])
        self.id = self.comm.sql_add_to_table(self.mydb, 'global_robot_list', robot)

        #self.task_allocation = TaskAllocation(self)
        #self.routing = Routing(self)
        #self.resource_management = ResourceManagement(self)
        self.action = Action(self)

        # Processes
        try:
            print("\nAGV " + str(self.id) + ":        Started")
            x = threading.Thread(target=TaskAllocation, args=(self,))
            x.start()
            y = threading.Thread(target=self.main)
            y.start()
            x.join()
            y.join()
        finally:
            print("\nAGV " + str(self.id) + ":        Stopped")
            self.__del__()

    def __del__(self):

        # Remove robot from database
        self.comm.sql_delete_from_table(self.mydb, 'global_robot_list', 'id', self.id)

    def main(self):

        while True:

            # Wait for a task on the local task list
            print("Agv " + str(self.id) + ":        Waiting for tasks...")
            while self.task_executing.id == -1:
                items = self.comm.sql_get_task_to_execute(self.mydb, self.id)
                for row in items:
                    self.task_executing = Task(row)
                time.sleep(1)

            # Start task
            print("Agv " + str(self.id) + ":        Start executing task " + str(self.task_executing.id))
            if self.status != 'EMPTY':
                self.status = 'BUSY'
            self.update_global_robot_list()

            # Remove from local task list and add task to executing list
            self.comm.sql_update_task(self.mydb, self.task_executing.id, self.id, 'executing', self.task_executing.message, self.task_executing.priority)

            # Go to task
            self.execute_task(self.task_executing)

            # Perform task
            self.action.pick()
            print("Agv " + str(self.id) + ":        Picked item of task " + str(self.task_executing.id))

            # Task executed
            self.comm.sql_update_task(self.mydb, self.task_executing.id, self.id, 'done', self.task_executing.message, self.task_executing.priority)
            self.task_executing = Task([-1, None])

            # Set status to IDLE when task is done or when done charging
            if self.status != 'EMPTY':
                self.status = 'IDLE'
            self.update_global_robot_list()

    def execute_task(self, task):

        # Compute dmas path towards task destination
        if self.routing_approach:
            if task.order_number in self.reserved_paths.keys():
                self.path = self.reserved_paths[task.order_number]
                self.slots = self.reserved_slots[task.order_number]
            else:
                self.path, self.slots, _ = self.routing.dmas(self.node, [task.pos_A], self.env.now)
        else:
            self.path, _ = find_shortest_path(self.graph, self.node, task.node)

        # Update state
        self.update_global_robot_list()

        #print("\nAgv: " + str(self.id) + ' executing task ' + str(task.node))
        #print("Path to follow: " + str([node for node in self.path]))
        #print("Slots to follow: " + str([slot for slot in self.slots]))

        # Move from node to node
        while len(self.path) > 0:
            if self.routing_approach:
                node_arriving_time = self.slots[0][0]
                if node_arriving_time > self.env.now:
                    self.env.timeout(node_arriving_time - self.env.now)
            self.action.move_to_node(self.path[0])

        # Check if task is charging task
        if task.id == '000':

            # Compute charging time
            if self.routing_approach:
                charging_time = task.priority
            else:
                charging_time = (100 - self.battery_status) / self.resource_management.charging_factor

            # Update status
            print("Agv " + str(self.ID) + ":      Is charging for " + str(charging_time) + " seconds")
            self.status = 'CHARGING'
            self.update_global_robot_list()

            # Charging
            time.sleep(charging_time)

            # Update robot status
            self.battery_status = self.battery_status + charging_time * self.resource_management.charging_factor
            self.status = 'IDLE'
            self.charged_time += int(charging_time) + calculate_path_traveltime(self.graph, self.path, self.speed)
            self.update_global_robot_list()

    def update_global_robot_list(self):
        robot = Robot([self.id, self.ip, self.port, self.x_loc, self.y_loc, self.theta, self.node,
                self.status, self.battery_status, self.travelled_time, self.charged_time,
                self.congestions, self.task_executing.id, str(self.path), str(self.total_path)])
        self.comm.sql_update_robot(self.mydb, robot)

    def search_closest_node(self, loc):
        node = min(self.graph.nodes.values(), key=lambda node: self.calculate_euclidean_distance(node.pos, loc))
        return node.name

    @staticmethod
    def calculate_euclidean_distance(a, b):
        return math.sqrt(math.pow(b[0] - a[0], 2) + math.pow(b[1] - a[1], 2))
