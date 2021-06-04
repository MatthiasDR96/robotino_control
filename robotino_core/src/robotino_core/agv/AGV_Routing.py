import math

import numpy as np

from robotino_core.solvers.feasibility_ant_solver import *


class Routing:
    """
            A class containing the intelligence of the Routing agent
    """

    def __init__(self, agv):

        # agv
        self.agv = agv

        # Process
        if self.agv.routing_approach == 'dmas':
            self.dmas_updater = self.agv.env.process(self.dmas_updater())
        self.collision_monitor = self.agv.env.process(self.collision_monitor())
        # self.homing = self.agv.env.process(self.homing())

    def dmas_updater(self):

        while True:

            # Timeout
            yield self.agv.env.timeout(self.agv.dmas_update_rate)  # Sample time

            # Get tasks in local task list
            local_task_list = [task for task in self.agv.kb['local_task_list_R' + str(self.agv.ID)].items]

            # Get start node of robot
            start_node = self.agv.task_executing.pos_A if self.agv.task_executing else self.agv.robot_node

            # Get start time of robot
            if self.agv.task_executing:
                if self.agv.status == 'CHARGING':
                    start_time = self.agv.env.now + self.agv.dmas_update_rate  # To compensate for unknown charging end
                else:
                    start_time = self.agv.slots[-1][0] + self.agv.slots[-1][1]
            else:
                start_time = self.agv.env.now

            # Update paths
            total_path = []
            for task in local_task_list:

                # Do dmas
                best_path, best_slots, best_delay = self.dmas(start_node, [task.pos_A], start_time)

                if best_path:
                    # Set paths towards all tasks
                    self.agv.reserved_paths[task.order_number] = best_path
                    self.agv.reserved_slots[task.order_number] = best_slots

                    # Set total planned path
                    total_path.extend(best_path)

                    # Start node for next task is end node of previous task
                    start_node = task.pos_A

                    # Starting time for next task
                    start_time = best_slots[-1][0] + best_slots[-1][1]

                    # Reserve slots
                    self.intent(self.agv.ID, best_path, best_slots)

            # Update total path
            self.agv.total_path = self.agv.path + total_path

    def dmas(self, start_node, nodes_to_visit, start_time):

        # Feasibility ants
        global_feasible_path, local_feasible_paths = self.think(start_node, nodes_to_visit)

        if local_feasible_paths:

            # Exploration ants
            explored_paths, fitness_values, total_edge_costs, total_delays, slots = self.explore(self.agv.ID,
                                                                                                 local_feasible_paths,
                                                                                                 start_time)

            # Best route selection
            best_path = explored_paths[int(np.argmin(fitness_values))]
            best_slots = slots[int(np.argmin(fitness_values))]
            best_delay = total_delays[int(np.argmin(fitness_values))]

            return best_path[1:], best_slots, best_delay

        else:
            print("No feasible paths found")
            return None, None, None

    def think(self, start_node, nodes_to_visit):
        global_best_solution, local_best_solutions = feasibility_ant_solve(self.agv.kb['graph'], start_node,
                                                                           nodes_to_visit)
        return global_best_solution, local_best_solutions

    def explore(self, agv_id, paths, start_time):

        # Init
        fitness_values = []
        total_travel_costs = []
        total_delays = []
        all_slots = []

        # Explore paths
        for path in paths:

            # Init
            timestamp = start_time
            total_delay = 0
            total_travel_time = 0
            slots = []

            # Calculate slot of nodes in between
            for i in range(0, len(path) - 1):

                # Calculate traveltime to drive to node i+1
                travel_time = self.agv.kb['graph'].edges[path[i], path[i + 1]].length / self.agv.robot_speed
                wanted_slot = (timestamp, travel_time)

                # Check available slots for node i+1
                slot, delay = self.agv.kb['graph'].nodes[path[i+1]].environmental_agent.check_slot(wanted_slot, agv_id)

                # Append slot and update state
                slots.append(slot)
                total_travel_time += travel_time
                total_delay += delay
                timestamp += travel_time + delay

            # Collect results
            fitness_values.append(timestamp)
            total_travel_costs.append(total_travel_time)
            total_delays.append(total_delay)
            all_slots.append(slots)

        return paths, fitness_values, total_travel_costs, total_delays, all_slots

    def intent(self, agv_id, path, slots):

        for i in range(len(path)):
            # Destination node
            dest = path[i]

            # Wanted slot
            wanted_slot = slots[i]

            # Reserve slot
            self.agv.kb['graph'].nodes[dest].environmental_agent.reserve_slot(wanted_slot, agv_id)

    def collision_monitor(self):

        while True:

            # Timeout
            yield self.agv.env.timeout(2)  # Sample time

            # Get all robot positions
            robots = np.copy(self.agv.comm.sql_read(self.agv.kb['global_robot_list']))

            # Check collisions
            for robot in robots:
                if not robot.ID == self.agv.ID:
                    x_diff = robot.robot_location[0] - self.agv.robot_location[0]
                    y_diff = robot.robot_location[1] - self.agv.robot_location[1]
                    distance = math.sqrt(x_diff ** 2 + y_diff ** 2)
                    if distance <= self.agv.collision_threshold:
                        self.agv.congestions += 1
                        self.agv.update_global_robot_list()
                        # raise Exception("Robot " + str(robot.ID) + ' and robot ' + str(self.agv.ID)
                        # + ' are in collision!')

    def homing(self):

        while True:

            # Timeout
            yield self.agv.env.timeout(2)  # Sample time

            # Get tasks in local task list
            local_task_list = [task for task in self.agv.kb['local_task_list_R' + str(self.agv.ID)].items]

            # Add homing task if all work is done
            if len(
                    local_task_list) == 0 and not self.agv.task_executing and not self.agv.robot_node == self.agv.home_task.pos_A:

                # Get path and slots
                if self.agv.routing_approach == 'dmas':
                    self.agv.reserved_paths[self.agv.home_task.order_number], \
                    self.agv.reserved_slots[self.agv.home_task.order_number], _ = \
                        self.agv.routing.dmas(self.agv.robot_node, [self.agv.home_task.pos_A], self.agv.env.now)

                # Add task to task list
                self.agv.comm.sql_write(self.agv.kb['local_task_list_R' + str(self.agv.ID)],
                                        self.agv.home_task)
