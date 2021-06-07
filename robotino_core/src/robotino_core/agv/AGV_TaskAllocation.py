from solvers.tsp_solver import tsp


class TaskAllocation:
    """
            A class containing the intelligence of the Task Allocation agent. This agent takes care of
            including new announced tasks into the robots task list
    """

    def __init__(self, agv):

        # agv
        self.agv = agv

    def main(self):

        while True:

            # Listen to incomming messages
            task = self.agv.comm3.tcp_server()
            print('\nAGV ' + str(self.agv.id) +'         Received task: ' + str(task))

            # Compute bid or add directly to local task list
            if task['message'] == 'announce':

                # Consider bidding when task limit is not reached
                bid_value = self.compute_bid(task)

                # Make bid structure
                bid = {'values': bid_value, 'robot': self.agv.id, 'task': task['id']}

                # Send bid to the auctioneer
                self.agv.comm3.tcp_client('localhost', 10002, bid)
                print("Agv " + str(self.agv.id) + ":      Sent bid " + str(bid['values']) + " to auctioneer")

            elif task['message'] == 'assign':

                # Add assigned tasks optimally to local task list
                self.update_local_task_list(task)
                print("Agv " + str(self.agv.id) + ":      Added task " + task.to_string() + " to local task list")

    ###
    # Compute bid
    ###

    def compute_bid(self, task):

        # Get tasks in local task list
        local_task_list = self.agv.comm3.sql_get_local_task_list(self.agv.id)

        # Get tasks in new local task list
        new_local_task_list = local_task_list + [task]

        # Get start node of robot
        start_node = self.agv.task_executing['node'] if not self.agv.task_executing['id'] == -1 else self.agv.node

        # Compute cost of current tour
        _, current_cost = self.compute_task_sequence_and_cost(local_task_list, start_node)

        # Compute cost of new tour
        _, new_cost = self.compute_task_sequence_and_cost(new_local_task_list, start_node)

        # Marginal cost to execute task
        min_sum = new_cost - current_cost
        min_max = new_cost
        return [min_sum, min_max]

    ###
    # Update local task list
    ###

    def update_local_task_list(self, task):

        # Get tasks in local task list (remove charging tasks)
        local_task_list = self.agv.comm3.sql_get_local_task_list(self.agv.id)

        # Get tasks in new local task list
        new_local_task_list = local_task_list + [task]

        # Get start node of robot
        start_node = self.agv.task_executing['node'] if not self.agv.task_executing['id'] == -1 else self.agv.node

        # Compute task sequence
        task_sequence, _ = self.compute_task_sequence_and_cost(new_local_task_list, start_node)

        # Add new local task list
        tasks = []
        priority = 1
        for task in task_sequence:
            tasks.append((self.agv.ID['id'], 'assigned', 'assign', priority, task['id']))
            priority += 1
                        
        # Assign task to agv task lists
        self.agv.comm3.sql_update_tasks(tasks)

    ###
    # Compute task sequence and cost
    ###

    def compute_task_sequence_and_cost(self, task_list, start_node):

        ###
        # Get optimal task sequence
        ###

        nodes_to_visit = [task['node'] for task in task_list]
        task_sequence_, _, cost = tsp(self.agv.graph, start_node, nodes_to_visit)
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

        return task_sequence, sum(cost)
