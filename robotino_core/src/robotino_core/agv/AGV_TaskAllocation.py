from robotino_core.datatypes.Bid import Bid
from robotino_core.datatypes.Task import Task
from robotino_core.solvers.optimal_insertion_solver import *


class TaskAllocation:
    """
            A class containing the intelligence of the Task Allocation agent. This agent takes care of
            including new announced tasks into the robots task list
    """

    def __init__(self, agv):

        # agv
        self.agv = agv

        # Process
        self.main = self.main()

    def main(self):

        while True:

            # Listen to incomming messages
            task = self.agv.comm.tcp_server()
            print('\nAGV ' + str(self.agv.id) +'         Received task: ' + task.to_string()) 

            # Compute bid or add directly to local task list
            if task.message == 'announce':

                # Consider bidding when task limit is not reached
                bid_value = self.compute_bid(task)

                # Make bid structure
                bid = Bid()
                bid.value = bid_value
                bid.robot = self.agv.robot
                bid.task = task

                # Send bid to the auctioneer
                self.agv.comm.tcp_client(self.agv.agv_to_fm_comm, bid)
                print("agv " + str(self.agv.id) + ":      Sent bid " + str(bid.value) + " to auctioneer")

            elif task.message == 'push':
                # Put task from FM directly onto local task list
                self.agv.comm.sql_update_task(task.id, self.agv.id, 'assigned', 'push', 1)

            else:
                # Add assigned tasks optimally to local task list
                self.update_local_task_list(task)
                print("agv " + str(self.agv.id) + ":      Added task " + task.to_string() + " to local task list")

    ###
    # Compute bid
    ###

    def compute_bid(self, task):

        # Get tasks in local task list
        local_task_list = [task for task in self.agv.kb['local_task_list_R' + str(self.agv.id)].items]

        # Get tasks in new local task list
        new_local_task_list = local_task_list + [task]

        # Get start node of robot
        start_node = self.agv.task_executing.pos_A if self.agv.task_executing else self.agv.robot_node

        # Compute cost of current tour
        _, current_cost = self.compute_task_sequence_and_cost(local_task_list, start_node)

        # Compute cost of new tour
        _, new_cost = self.compute_task_sequence_and_cost(new_local_task_list, start_node)

        # Marginal cost to execute task
        min_sum = new_cost - current_cost
        min_max = new_cost
        objective = (float(self.agv.epsilon) * min_sum + (1 - float(self.agv.epsilon)) * min_max)

        return objective

    ###
    # Update local task list
    ###

    def update_local_task_list(self, task):

        # Get tasks in local task list (remove charging tasks)
        local_task_list = [task for task in self.agv.kb['local_task_list_R' + str(self.agv.ID)].items if not task.order_number == '000']

        # Get tasks in new local task list
        new_local_task_list = local_task_list + [task]

        # Get start node of robot
        start_node = self.agv.task_executing.pos_A if self.agv.task_executing else self.agv.robot_node

        # Compute task sequence
        task_sequence, _ = self.compute_task_sequence_and_cost(new_local_task_list, start_node)

        # Delete local task list
        for i in range(len(self.agv.kb['local_task_list_R' + str(self.agv.ID)].items)):
            self.agv.kb['local_task_list_R' + str(self.agv.ID)].get()

        # Put optimal task_sequence in local task list
        for task in task_sequence:
            self.agv.comm.sql_write(self.agv.kb['local_task_list_R' + str(self.agv.ID)], task)

    ###
    # Compute task sequence and cost
    ###

    def compute_task_sequence_and_cost(self, task_list, start_node):

        ###
        # Get optimal task sequence
        ###

        nodes_to_visit = [task.pos_A for task in task_list]
        task_sequence_, tour, cost = optimal_insertion_solve(self.agv.kb['graph'], start_node, nodes_to_visit)
        task_sequence = [task_list[nodes_to_visit.index(name)] for name in task_sequence_]

        ###
        # Consider resource management
        ###

        if self.agv.charging_approach == 'optimal':
            charging_station, insertion_index, charging_time, charging_cost = self.agv.resource_management.solve(
                task_sequence_)
            if insertion_index is not None:
                task_sequence.insert(insertion_index, Task('000', charging_station, charging_time))
            cost += charging_cost

        return task_sequence, cost
