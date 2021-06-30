from robotino_core.solvers.tsp_solver import tsp
from robotino_core.Comm import Comm
import time
import pickle
import socket
from multiprocessing import Pool
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
			start_new_thread(self.handle_client, (conn, addr))

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
					
				# Update executing task
				task_dict = {'progress': progress}
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
			time.sleep(self.agv.params['local_consensus_rate'])

			# Get all robots in the fleet
			robots = comm.sql_select_everything_from_table('global_robot_list')

			# Remove this robot
			robots = [robot for robot in robots if robot['id'] != self.agv.id] if robots is not None else []

			# Iterate over all tasks in local task list
			combinations = []
			local_task_list = comm.sql_get_local_task_list(self.agv.id)
			if local_task_list is not None:
				for robot in robots:
					for task in local_task_list:
						task['message'] = "announce"
						combinations.append((robot['ip'], robot['port'], task))

			# Working pool
			self.p = Pool(processes=max(1, len(combinations)))
			bids = self.p.map(tcp_client, combinations)
			bids = [bid for bid in bids if bid]

			# If there are valid bids
			if len(bids) > 0: 
						
				# Resolution
				best_bid_dict = self.resolution_lb(bids)
				best_bid = best_bid_dict['values'][0] # self.agv.params['epsilon'] * best_bid_list['values'][0] + (1 - self.agv.params['epsilon']) * best_bid_list['values'][1]

				# My bid
				my_bid_list = self.compute_bid(local_task_list, task, True) # datetime.strptime(task['estimated_start_time'], '%H:%M:%S').time() 
				my_bid = my_bid_list[0] # self.agv.params['epsilon'] * my_bid_list[0] + (1 - self.agv.params['epsilon']) * my_bid_list[1]

				# If best bid is better than my bid
				if best_bid + my_bid < 0:

					print("Better solution found")

					# Send task to winning robot
					robot = best_bid_dict['robot']
					best_bid_dict['task']['message'] = 'assign'
					best_bid_dict['task']['robot'] = robot['id']
					comm.tcp_client(robot['ip'], robot['port'], best_bid_dict['task'])

	def resolution_lb(self, bids):
		bids.sort(key=lambda p: self.agv.params['epsilon'] * p['values'][0] + (1 - self.agv.params['epsilon']) * p['values'][1])
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
		print('AGV ' + str(self.agv.id) +'         Received task: ' + str(task['id']))

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
			print("Agv " + str(self.agv.id) + ":        Sent bid " + str(bid['values'][:2]) + " to auctioneer")

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

			# Compute estimated start, end and duration
			task_index = task_sequence.index(task)
			estimated_start_time = timedelta(seconds=sum(edges[0:task_index])) + datetime.now()
			estimated_end_time = timedelta(seconds=sum(edges[0:task_index+1])) + datetime.now()
			estimated_duration = estimated_end_time - estimated_start_time

			# Update task
			tasks.append((self.agv.id, 'assigned', 'assign', priority, estimated_start_time.strftime('%H:%M:%S'), estimated_end_time.strftime('%H:%M:%S'), str(estimated_duration), '-', '-', '-', 0, task['id']))
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

def tcp_client(x):

	ip = x[0]
	port = x[1]
	data = x[2]
	attempts = 0
	while True:

		try:

			# Create a TCP/IP socket
			sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

			# Connect the socket to the port where the server is listening
			sock.connect((ip, port))
					
			# Make message
			pickled_data = pickle.dumps(data)

			# Send data
			sock.sendall(pickled_data)

			# Look for the response
			rec_data = sock.recv(1024)
			rec_data_loaded = pickle.loads(rec_data)
			if not isinstance(rec_data_loaded, str):
				print("Central auctioneer:		Received bid " + str(rec_data_loaded['values'][:2]) + " from AGV " + str(rec_data_loaded['robot']['id']) + ' on task ' + str(rec_data_loaded['task']['id']) + ' at ' + str(datetime.now().time()))
			return rec_data_loaded

		except(socket.error):
			print("\nConnection failed from " + str(port) + " to " + str(port) + " , retrying...")
			attempts += 1
			if attempts > 3:
				return None
		except EOFError:
			print("\nReceived message from " + str(port) + " not correct, retrying...")
			attempts += 1
			if attempts > 3:
				return None
		finally:
			sock.close()