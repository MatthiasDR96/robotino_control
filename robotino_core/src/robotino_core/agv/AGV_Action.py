import time


class AGV_Action:

    def __init__(self, agv):

        # AGV
        self.agv = agv

    def move_to_pos_manual(self, node_position):
        cur_pos = (self.agv.x_loc, self.agv.y_loc)
        rho = self.agv.dist_euclidean(cur_pos, node_position)
        while rho > 0.1:
            cur_pos = (self.agv.x_loc, self.agv.y_loc)
            rho = self.agv.dist_euclidean(cur_pos, node_position)

    def move_to_pos_autonomous(self, node_position):
        
        # Temp pos
        tmp_x = self.agv.x_loc
        tmp_y = self.agv.y_loc

        # Interpolate
        for i in range(1, 11):

            # Stop
            if self.agv.exit_event.is_set():
               return False

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

    def cancel_goal(self):
        pass

    def pick(self):
        print("AGV-agent:	Picked item ")

    def place(self):
        print("AGV-agent:	Placed item ")
