from robotino_core.solvers.tsp_solver import tsp
from robotino_core.Comm import Comm
import time
import pickle
import numpy as np
from datetime import datetime, timedelta
from _thread import start_new_thread

class TaskAllocation:
	"""
			A class containing the intelligence of the Task Allocation agent. This agent takes care of
			including new announced tasks into the robots task list
	"""

	def __init__(self, agv):

		# AGV
		self.agv = agv

	def main(self):

		# Open database connection
		print("   Task allocation agent:     Started")
		comm = Comm(self.agv.ip, self.agv.port, self.agv.host, self.agv.user, self.agv.password, self.agv.database)
		comm.tcp_server_open()
		comm.sql_open()

		# Loop
		while True:

			# Wait for connection
			conn, addr = comm.sock_server.accept()

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

			# If task executing
			if not self.agv.task_executing['id'] == -1:

				# Calculate progress
				progress = (self.agv.dist_done / self.agv.dist_to_do) * 100 if not self.agv.dist_to_do == 0.0 else 0.0

				# Calculate estimated end time and duration
				distance_yet_to_do = self.agv.dist_to_do - self.agv.dist_done
				self.agv.estimated_end_time = datetime.now() + timedelta(seconds=distance_yet_to_do / self.agv.params['robot_speed']) # TODO possible delays
				self.agv.estimated_duration = self.agv.estimated_end_time - self.agv.start_time
					
				# Update executing task
				task_dict = {'progress': progress, 'estimated_end_time': self.agv.estimated_end_time.strftime('%H:%M:%S'), 'estimated_duration': str(self.agv.estimated_duration)}
				comm.sql_update_task(self.agv.task_executing['id'], task_dict)

			# Close thread at close event 
			if self.agv.exit_event.is_set():
				break

	def local_consensus(self):

		# Open database connection
		comm = Comm(self.agv.ip, self.agv.port, self.agv.host, self.agv.user, self.agv.password, self.agv.database)
		comm.sql_open()

		# Loop
		while True:

			# Timeout
			time.sleep(2)

			# Get all robots in the fleet
			robots = comm.sql_select_everything_from_table('global_robot_list')

			# Remove this robot
			if robots is not None: robots = [robot for robot in robots if robot['id'] != self.agv.id]

			# Iterate over all tasks in local task list
			local_task_list = comm.sql_get_local_task_list(self.agv.id)
			if local_task_list is not None:
				for task in local_task_list:

					# Announce
					task['message'] = "announce"

					# Announce tasks and receive bids
					bids = []
					for robot in robots:
						# print("Announce task " + str(task['id']) + ' to robot ' + str(robot['id']))
						data = comm.tcp_client(robot['ip'], robot['port'], task)
						if data is not None: bids.append(data)

					# If there are valid bids
					if bids: 
						
						# Resolution
						best_bid = self.resolution_lb(bids) 

						# My bid
						#my_bid = datetime.strptime(task['estimated_start_time'], '%H:%M:%S').time() 
						my_bid = self.compute_bid(local_task_list, task, True)

						# If best bid is better than my bid
						if best_bid['values'][0] < my_bid[0]:

							# Send task to winning robot
							robot = best_bid['robot']
							best_bid['task']['message'] = 'assign'
							best_bid['task']['robot'] = robot['id']
							comm.tcp_client(robot['ip'], robot['port'], best_bid['task'])

	def resolution_lb(self, bids):
		bids.sort(key=lambda p: p['values'][2]) # Minimize start time of task
		best_bid = bids[0]
		return best_bid		
				
	def handle_client(self, conn, _):

		# Open database connection
		comm = Comm(self.agv.ip, self.agv.port, self.agv.host, self.agv.user, self.agv.password, self.agv.database)
		comm.sql_open()

		# Receive message
		data = conn.recv(1024)
						
		# Convert data to dictionary
		task = pickle.loads(data)
		#print('\nAGV ' + str(self.agv.id) +'         Received task: ' + str(task['id']))

		# Get local task list
		local_task_list = comm.sql_get_local_task_list(self.agv.id)

		# Compute bid or add directly to local task list
		if task['message'] == 'announce':

			# Compute bid
			bid_value_list = self.compute_bid(local_task_list, task, False)

			# Make bid structure
			robot = {'id': self.agv.id, 'ip': self.agv.ip, 'port': self.agv.port}
			bid = {'values': bid_value_list, 'robot': robot, 'task': task}

			# Send bid to the auctioneer
			conn.sendall(pickle.dumps(bid))
			# print("Agv " + str(self.agv.id) + ":        Sent bid " + str(bid['values'][:2]) + " to auctioneer")

		elif task['message'] == 'assign':

			# Add assigned tasks optimally to local task list
			updated_tasks = self.update_local_task_list(local_task_list, task)
			result = comm.sql_update_tasks(updated_tasks)
			if result:
				conn.sendall(pickle.dumps('task_accepted'))
				# print("Agv " + str(self.agv.id) + ":        Added task " + str(task['id']) + " to local task list")

		# Close connection
		conn.close()
		comm.sql_close()

	###
	# Compute bid
	###

	def compute_bid(self, local_task_list, task, substract):

		# Get tasks in new local task list
		if not substract:
			new_local_task_list = (local_task_list if local_task_list is not None else []) + [task]
		else:
			new_local_task_list = [task_ for task_ in local_task_list if task_['id'] != task['id']]

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
		if not substract:
			task_index = new_sequence.index(task)
			start_time = (timedelta(seconds=sum(new_edges[0:task_index])) + datetime.now())
			end_time = (timedelta(seconds=sum(new_edges[0:task_index+1])) + datetime.now())
			duration = end_time - start_time
		else:
			start_time = 0.0
			end_time = 0.0
			duration = 0.0

		# Objective list
		objective_list = [min_sum, min_max, start_time, end_time, duration]

		return objective_list

	###
	# Update local task list
	###

	def update_local_task_list(self, local_task_list, task):

		# Get tasks in new local task list
		new_local_task_list =  (local_task_list if local_task_list is not None else []) + [task]

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

		return tasks

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
