import rospy
import math
import yaml

from robotino_core.Comm import Comm
from robotino_core.Graph import Graph
from robotino_core.agv.AGV_Action_ROS import Action
from robotino_core.agv.AGV_Routing import Routing
from robotino_core.agv.AGV_TaskAllocation import TaskAllocation
from robotino_core.agv.AGV_ResourceManagement import ResourceManagement
from robotino_core.solvers.astar_solver import find_shortest_path

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class AGV:
    """
        A class containing the intelligence of the agv agent
    """

    def __init__(self, ip, port, id, host, user, password, database):

        # Init communication
        self.ip = ip
        self.port = port
        self.comm1 = Comm(ip, port, host, user, password, database)
        self.comm2 = Comm(ip, port, host, user, password, database)

        # Read params
        with open(r'params/setup.yaml') as file:
            self.params = yaml.load(file, Loader=yaml.FullLoader)

        # Create graph
        self.graph = Graph()
        self.node_names = self.params['node_names']
        self.node_locations = self.params['node_locations']
        self.graph.create_nodes(self.node_locations, list(self.node_names.keys()))
        self.graph.create_edges(list(self.node_names.keys()), list(self.node_names.values()))

        # Agv attributes
        self.id = id
        self.speed = self.params['robot_speed']
        self.task_execution_time = self.params['task_execution_time']
        self.battery_threshold = self.params['battery_threshold']
        self.collision_threshold = self.params['collision_threshold']
        self.max_charging_time = self.params['max_charging_time']
        self.max_tasks_in_task_list = self.params['max_tasks_in_task_list']
        self.initial_resources = self.params['initial_resources']
        self.depot_locations = self.params['depot_locations']  

        # Control types
        self.charging_approach = False
        self.routing_approach = False

        # Updated attributes
        self.x_loc = 0.0
        self.y_loc = 0.0
        self.theta = 0.0
        self.node = self.search_closest_node((self.x_loc, self.y_loc))
        self.status = 'IDLE'
        self.battery_status = 100.0
        self.travelled_time = 0.0
        self.charged_time = 0.0
        self.congestions = 0
        self.task_executing = {'id': -1, 'node': None}
        self.path = []
        self.total_path = []
        
        # Add robot to database
        robot_dict = {"id": self.id, "ip": self.ip, "port": self.port, "x_loc": self.x_loc, "y_loc": self.y_loc, "theta": self.theta, "node": self.node,
                "status": self.status, "battery_status": self.battery_status, "travelled_time": self.travelled_time, "charged_time": self.charged_time,
                "congestions": self.congestions, "task_executing": self.task_executing['id'], "path": str(self.path), "total_path": str(self.total_path)}
        self.comm1.sql_add_to_table('global_robot_list', robot_dict)

        #self.task_allocation = TaskAllocation(self)
        #self.routing = Routing(self)
        #self.resource_management = ResourceManagement(self)
        self.action = Action(self)

        # Init subscriber
        self.status_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Processes
        self.main()

        # Shutdown
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        # Remove robot from database
        if self.id is not None:
            rospy.loginfo("Agv " + str(self.id) + ":        Shutting down")
            self.comm2.sql_delete_from_table('global_robot_list', 'id', self.id)
            del self.comm1
            del self.comm2

    def main(self):

        rospy.loginfo("Agv " + str(self.id) + ":        Started")
        while not rospy.is_shutdown():

            # Wait for a task on the local task list
            items = self.comm2.sql_get_task_to_execute(self.id)
            for row in items:
                self.task_executing = row
            
            # Execute task
            if not self.task_executing['id'] == -1:

                # Start task
                rospy.loginfo("Agv " + str(self.id) + ":        Start executing task " + str(self.task_executing['id']))
                if self.status != 'EMPTY': self.status = 'BUSY'

                # Remove from local task list and add task to executing list
                task_dict = {'robot': self.id, 'status': 'executing', 'message': self.task_executing['message'], 'priority': self.task_executing['priority']}
                self.comm2.sql_update_task(self.task_executing['id'], task_dict)

                # Go to task
                self.execute_task(self.task_executing)

                # Perform task
                self.action.pick()
                rospy.loginfo("Agv " + str(self.id) + ":        Picked item of task " + str(self.task_executing['id']))

                # Task executed
                task_dict = {'robot': self.id, 'status': 'done', 'message': self.task_executing['message'], 'priority': self.task_executing['priority']}
                self.comm2.sql_update_task(self.task_executing['id'], task_dict)
                self.task_executing = {'id': -1, 'node': None}

                # Set status to IDLE when task is done or when done charging
                if self.status != 'EMPTY': self.status = 'IDLE'

    def execute_task(self, task):

        # Compute dmas path towards task destination
        self.path, _ = find_shortest_path(self.graph, self.node, task['node'])

        # Move from node to node
        while len(self.path) > 0:
            result = self.action.move_to_node(self.path[0])
            if result: rospy.loginfo("Goal execution done!")

    def update_global_robot_list(self):
        robot_dict = {"id": self.id, "x_loc": self.x_loc, "y_loc": self.y_loc, "theta": self.theta, "node": self.node,
                "status": self.status, "battery_status": self.battery_status, "travelled_time": self.travelled_time, "charged_time": self.charged_time,
                "congestions": self.congestions, "task_executing": self.task_executing['id'], "path": str(self.path), "total_path": str(self.total_path)}
        self.comm1.sql_add_to_table('global_robot_list', robot_dict)
        self.comm1.sql_update_robot(self.id, robot_dict)

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

        # Search closest node
        self.node = self.search_closest_node((self.x_loc, self.y_loc))

        # Update robot
        self.update_global_robot_list()