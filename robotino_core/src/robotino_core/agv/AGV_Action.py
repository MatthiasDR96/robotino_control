import time


class AGV_Action:

    def __init__(self, agv):

        # AGV
        self.agv = agv

    def move_to_pos(self, node_position):
        
        # Temp pos
        tmp_x = self.agv.x_loc
        tmp_y = self.agv.y_loc

        # Interpolate
        for i in range(1, 11):

            # New location
            x = tmp_x + i/10 * (node_position[0] - tmp_x)
            y = tmp_y + i/10 * (node_position[1] - tmp_y)

            # Get distance
            dist = self.agv.dist_euclidean((self.agv.x_loc, self.agv.y_loc), (x, y))

            # Sleep
            cost = dist / self.agv.params['robot_speed']
            time.sleep(cost)

            # Update location
            self.agv.x_loc = float(x)
            self.agv.y_loc = float(y)
            self.agv.traveled_dist += dist
            self.agv.task_executing_dist_done += dist

            # Update battery status
            self.agv.battery_status = self.agv.battery_status - cost * self.agv.params['resource_scale_factor']

        # Update status at node
        self.agv.x_loc = float(node_position[0])
        self.agv.y_loc = float(node_position[1])

        return True

    def pick(self):
        print("Agv " + str(self.agv.id) + ":         Picked item ")

    def place(self):
        print("Agv " + str(self.agv.id) + ":         Placed item ")
