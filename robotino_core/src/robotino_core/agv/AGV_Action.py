import time


class Action:

    def __init__(self, agv):
        self.agv = agv

    def move_to_node(self, node):
        
        print("AGV " + str(self.agv.id) + ":        Move to node " + node)
        node_position = self.agv.graph.nodes[node].pos
        time.sleep(2)

        self.agv.node = node
        self.agv.x_loc = node_position[0]
        self.agv.y_loc = node_position[1]
        self.agv.path = self.agv.path[1:]
        # self.agv.slots = self.agv.slots[1:]
        print("AGV " + str(self.agv.id) + ":        Moved to node " + node)

    def pick(self):
        time.sleep(5)
        print("AGV " + str(self.agv.id) + ":        Pick")

    def place(self):
        print("AGV " + str(self.agv.id) + ":        Place")