import time


class Action:

    def __init__(self, agv):

        # AGV
        self.agv = agv

    def move_to_node(self, node):
        
        # Get node position
        print("AGV " + str(self.agv.id) + ":        Move to node " + node)
        node_position = self.agv.graph.nodes[node].pos

        # Interpolate
        for i in range(1, 11):

            # New location
            x = self.agv.x_loc + i/10 * (node_position[0] - self.agv.x_loc)
            y = self.agv.y_loc + i/10 * (node_position[1] - self.agv.y_loc)

            # Get distance
            distance = self.agv.calculate_euclidean_distance((self.agv.x_loc, self.agv.y_loc), (x, y))
            #self.agv.traveled_cost += distance

            # Sleep
            time.sleep((distance/self.agv.params['robot_speed']))

            # Update location
            self.agv.x_loc = float(x)
            self.agv.y_loc = float(y)

            # Update battery status
            self.agv.battery_status = self.agv.battery_status - 1

        # Update status at node
        self.agv.node = node
        self.agv.x_loc = float(node_position[0])
        self.agv.y_loc = float(node_position[1])
        print("AGV " + str(self.agv.id) + ":        Moved to node " + node)
        

        return True

    def pick(self):
        print("Agv " + str(self.agv.id) + ":        Picked item of task " + str(self.agv.task_executing['id']))

    def place(self):
        print("Agv " + str(self.agv.id) + ":        Placed item of task " + str(self.agv.task_executing['id']))
