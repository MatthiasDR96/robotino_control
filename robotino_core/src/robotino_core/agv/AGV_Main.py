import rospy
import math

from robotino_core.Comm import Comm
from robotino_core.Graph import Graph
from robotino_core.datatypes.Task import Task
from robotino_core.datatypes.Robot import Robot
from robotino_core.agv.AGV_Action import Action
from robotino_core.solvers.astar_solver import find_shortest_path

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion



class AGV:
    """
        A class containing the intelligence of the agv agent
    """

    def __init__(self):

        # Communication attributes
        self.ip = 0 #rospy.get_param("/robotino_core_node/robot_ip")
        self.port = 0 #rospy.get_param("/robotino_core_node/robot_port")
        # self.comm = Comm(self.ip, self.port)

        # Init subscriber
        self.status_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Agv attributes
        self.id = None

        # Create graph
        self.node_locations = (10, 30),(10, 50),(10, 70),(30, 10),(30, 30),(30, 50),(30, 70),(30, 90),(60, 10),(60, 30),(60, 50),(60, 70),(60, 90),(90, 10),(90, 50),(90, 90)
        self.node_neighbors = [["pos_5"], ["pos_6"], ["pos_7"], ["pos_5", "pos_9"], ["pos_1", "pos_4", "pos_6"],
                 ["pos_2", "pos_5", "pos_7", "pos_10", "pos_11", "pos_12"],
                 ["pos_3", "pos_6", "pos_8"], ["pos_7", "pos_13"], ["pos_4", "pos_10", "pos_14"],
                 ["pos_6", "pos_9", "pos_11"],
                 ["pos_6", "pos_10", "pos_12", "pos_15"], ["pos_6", "pos_11", "pos_13"],
                 ["pos_8", "pos_12", "pos_16"], ["pos_15", "pos_9"], ["pos_11", "pos_14", "pos_16"],
                 ["pos_13", "pos_15"]]
        self.node_names = ["pos_1", "pos_2", "pos_3", "pos_4", "pos_5", "pos_6", "pos_7", "pos_8", "pos_9", "pos_10",
             "pos_11", "pos_12", "pos_13", "pos_14", "pos_15", "pos_16"]
        self.graph = Graph()
        self.graph.create_nodes(self.node_locations, self.node_names)
        self.graph.create_edges(self.node_names, self.node_neighbors)

        # Updated attributes
        self.x_loc = 0.0
        self.y_loc = 0.0
        self.theta = 0.0
        self.node = self.search_closest_node([self.x_loc, self.y_loc])
        self.status = 'IDLE'
        self.battery_status = 100.0
        self.travelled_time = 0.0
        self.charged_time = 0.0
        self.congestions = 0
        self.task_executing = Task([-1, None])
        self.path = []
        self.total_path = []

        # Add robot to database
        robot = Robot([None, self.ip, self.port, self.x_loc, self.y_loc, self.theta, self.node,
                self.status, self.battery_status, self.travelled_time, self.charged_time,
                self.congestions, self.task_executing.id, str(self.path), str(self.total_path)])
        # self.id = self.comm.sql_add_to_table(self.mydb, 'global_robot_list', robot)
        

        # Action layer
        self.action = Action(self)

        # Processes
        self.main()

    def __del__(self):

        # Remove robot from database
        # self.comm.sql_delete_from_table(self.mydb, 'global_robot_list', 'id', self.id)
        pass

    def main(self):

        while not rospy.is_shutdown():
            
            # Wait for a task on the local task list
            # while self.task_executing.id == -1:
                # items = self.comm.sql_get_task_to_execute(self.mydb, self.id)
                # for row in items:
                    # self.task_executing = Task(row)
                # time.sleep(1)

            # Start task
            if self.status != 'EMPTY': self.status = 'BUSY'
            # self.update_global_robot_list()

            # Remove from local task list and add task to executing list
            # self.comm.sql_update_task(self.mydb, self.task_executing.id, self.id, 'executing', self.task_executing.message, self.task_executing.priority)

            # Go to task
            self.task_executing = Task([1, 'pos_16'])
            self.execute_task(self.task_executing)

            # Perform task
            # self.action.pick()

            # Task executed
            # self.comm.sql_update_task(self.mydb, self.task_executing.id, self.id, 'done', self.task_executing.message, self.task_executing.priority)
            self.task_executing = Task([-1, None])

            # Set status to IDLE when task is done or when done charging
            if self.status != 'EMPTY': self.status = 'IDLE'
            # self.update_global_robot_list()

    def execute_task(self, task):

        # Compute path towards task destination
        self.path, _ = find_shortest_path(self.graph, self.node, task.node)
        # self.update_global_robot_list()

        # Move from node to node
        while len(self.path) > 0:
            result = self.action.move_to_node(self.path[0])
            if result: rospy.loginfo("Goal execution done!")

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

    def odom_callback(self, msg):

        # Get position
        self.x_loc = msg.pose.pose.position.x
        self.y_loc = msg.pose.pose.position.y

        # Get orientation
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion (orientation_list)
        self.theta = yaw