from robotino_core.solvers.tsp_solver import tsp
from robotino_core.Comm import Comm
import pickle
import numpy as np
from datetime import datetime, timedelta

class TaskAllocation:
	"""
			A class containing the intelligence of the Task Allocation agent. This agent takes care of
			including new announced tasks into the robots task list
	"""

	def __init__(self, agv):

		# agv
		self.agv = agv

	def main(self):

		# Open database connection
		print("   Task allocation agent:     Started")
		self.comm = Comm(self.agv.ip, self.agv.port, self.agv.host, self.agv.user, self.agv.password, self.agv.database)
		self.comm.tcp_server_open()
		self.comm.sql_open()

		# Loop
		while True:

			# Wait for connection
			conn, addr = self.comm.sock_server.accept()

			# Handle in separate thread
			self.handle_client(conn, addr)

			# Close thread at close event 
			if self.agv.exit_event.is_set():
				break

	def progress_tracker(self):

		# Open database connection
		comm = Comm(self.agv.ip, self.agv.port, self.agv.host, self.agv.user, self.agv.password, self.agv.database)
		comm.sql_open()

		# Loop
		while True:
			if not self.agv.task_executing['id'] == -1:
				if not self.agv.dist_to_do == 0.0:
					progress = (self.agv.dist_done / self.agv.dist_to_do) * 100
					task_dict = {'progress': progress}
					comm.sql_update_task(self.agv.task_executing['id'], task_dict)

			# Close thread at close event 
			if self.agv.exit_event.is_set():
				break
				

	def handle_client(self, conn, _):

		# Receive message
		data = conn.recv(1024)
						
		# Convert data to dictionary
		task = pickle.loads(data)
		print('\nAGV ' + str(self.agv.id) +'         Received task: ' + str(task['id']))

		# Compute bid or add directly to local task list
		if task['message'] == 'announce':

			# Consider bidding when task limit is not reached
			bid_value_list = self.compute_bid(task)

			# Make bid structure
			robot = {'id': self.agv.id, 'ip': self.agv.ip, 'port': self.agv.port}
			bid = {'values': bid_value_list, 'robot': robot, 'task': task}

			# Send bid to the auctioneer
			conn.sendall(pickle.dumps(bid))
			print("Agv " + str(self.agv.id) + ":        Sent bid " + str(bid['values'][:2]) + " to auctioneer")

		elif task['message'] == 'assign':

			# Add assigned tasks optimally to local task list
			result = self.update_local_task_list(task)
			if result:
				conn.sendall(pickle.dumps('task_accepted'))
				print("Agv " + str(self.agv.id) + ":        Added task " + str(task['id']) + " to local task list")

		# Close connection
		conn.close()

	###
	# Compute bid
	###

	def compute_bid(self, task):

		# Get tasks in local task list
		local_task_list = self.comm.sql_get_local_task_list(self.agv.id)

		# Get tasks in new local task list
		new_local_task_list = local_task_list + [task]

		# Get start node of robot
		start_node = self.agv.task_executing['node'] if not self.agv.task_executing['id'] == -1 else self.agv.node

		# Compute cost of current tour
		_, _, current_cost = self.compute_task_sequence_and_cost(local_task_list, start_node)

		# Compute cost of new tour
		new_sequence, new_edges, new_cost = self.compute_task_sequence_and_cost(new_local_task_list, start_node)

		# Marginal cost to execute task
		min_sum = new_cost - current_cost
		min_max = new_cost

		# Compute start, end and duration
		task_index = new_sequence.index(task)
		start_time = timedelta(seconds=sum(new_edges[0:task_index])) + datetime.now()
		end_time = timedelta(seconds=sum(new_edges[0:task_index+1])) + datetime.now()
		duration = end_time - start_time

		# Objective list
		objective_list = [min_sum, min_max, start_time, end_time, duration]

		return objective_list

	###
	# Update local task list
	###

	def update_local_task_list(self, task):

		# Get tasks in local task list (remove charging tasks)
		local_task_list = self.comm.sql_get_local_task_list(self.agv.id)

		# Get tasks in new local task list
		new_local_task_list = local_task_list + [task]

		# Get start node of robot
		start_node = self.agv.task_executing['node'] if not self.agv.task_executing['id'] == -1 else self.agv.node

		# Compute task sequence
		task_sequence, edges, _ = self.compute_task_sequence_and_cost(new_local_task_list, start_node)

		# Add new local task list
		tasks = []
		priority = 1
		for task in task_sequence:

			# Compute start, end and duration
			task_index = task_sequence.index(task)
			start_time = timedelta(seconds=sum(edges[0:task_index])) + datetime.now()
			end_time = timedelta(seconds=sum(edges[0:task_index+1])) + datetime.now()
			duration = end_time - start_time

			# Update task
			tasks.append((self.agv.id, 'assigned', 'assign', priority, start_time.strftime('%H:%M:%S'), end_time.strftime('%H:%M:%S'), str(duration), '-', '-', '-', 0, task['id']))
			priority += 1
						
		# Assign task to agv task lists
		result = self.comm.sql_update_tasks(tasks)

		return result

	###
	# Compute task sequence and cost
	###

	def compute_task_sequence_and_cost(self, task_list, start_node):

		###
		# Get optimal task sequence
		###

		nodes_to_visit = [task['node'] for task in task_list]
		task_sequence_, _, edges = tsp(self.agv.graph, start_node, nodes_to_visit)
		task_sequence = [task_list[nodes_to_visit.index(name)] for name in task_sequence_]
		edges = np.array(edges) / self.agv.params['robot_speed']
		cost = sum(edges)

		###
		# Consider resource management
		###

		# if self.agv.charging_approach == 'optimal':
			# charging_station, insertion_index, charging_time, charging_cost = self.agv.resource_management.solve(
				# task_sequence_)
			# if insertion_index is not None:
				# task_sequence.insert(insertion_index, Task('000', charging_station, charging_time))
			# cost += charging_cost

		return task_sequence, edges, cost
