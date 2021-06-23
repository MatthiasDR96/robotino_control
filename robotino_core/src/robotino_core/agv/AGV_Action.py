import time


class Action:

    def __init__(self, agv):

        self.agv = agv

    def move_to_node(self, node):
        
        # Get node
        print("AGV " + str(self.agv.id) + ":        Move to node " + node)
        node_position = self.agv.graph.nodes[node].pos

        # Get distance
        distance = self.agv.calculate_euclidean_distance((self.agv.x_loc, self.agv.y_loc), node_position)
        self.agv.dist_done += distance

        # Sleep
        time.sleep((distance/self.agv.params['robot_speed']))

        # Update status at node
        self.agv.battery_status = self.agv.battery_status - 10
        self.agv.node = node
        self.agv.x_loc = node_position[0]
        self.agv.y_loc = node_position[1]
        self.agv.current_path = self.agv.current_path[1:]
        self.agv.current_slots = self.agv.current_slots[1:]
        print("AGV " + str(self.agv.id) + ":        Moved to node " + node)

        return True

    def pick(self):
        print("Agv " + str(self.agv.id) + ":        Picked item of task " + str(self.agv.task_executing['id']))

    def place(self):
        print("Agv " + str(self.agv.id) + ":        Placed item of task " + str(self.agv.task_executing['id']))
