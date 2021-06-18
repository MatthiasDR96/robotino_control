import time


class Action:

    def __init__(self, agv):

        self.agv = agv

    def move_to_node(self, node):
        
        print("AGV " + str(self.agv.id) + ":        Move to node " + node)
        node_position = self.agv.graph.nodes[node].pos
        distance = self.agv.calculate_euclidean_distance((self.agv.x_loc, self.agv.y_loc), node_position)
        self.agv.dist_done += distance
        time.sleep(distance/self.agv.params['robot_speed'])
        self.agv.battery_status = self.agv.battery_status - 10
        self.agv.node = node
        self.agv.x_loc = node_position[0]
        self.agv.y_loc = node_position[1]
        self.agv.path = self.agv.path[1:]
        self.agv.slots = self.agv.slots[1:]
        print("AGV " + str(self.agv.id) + ":        Moved to node " + node)

    def pick(self):
        print("AGV " + str(self.agv.id) + ":        Pick")

    def place(self):
        print("AGV " + str(self.agv.id) + ":        Place")