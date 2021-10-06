from datetime import datetime, timedelta
import time
import pickle
import socket
from multiprocessing import Pool
from _thread import start_new_thread
import os
import yaml

from robotino_core.Comm import Comm
from robotino_core.solvers.tsp_solver import *
from robotino_core.solvers.astar_solver import *

class AGV_TA_agent:

	"""
			A class containing the intelligence of the Task Allocation agent. This agent takes care of
			including new announced tasks into the robots task list
	"""

	def __init__(self, params_file):

		# Read params
		this_dir = os.path.dirname(os.path.dirname(__file__))
		data_path = os.path.join(this_dir, "params", params_file)
		with open(data_path, 'r') as file:
			self.params = yaml.load(file, Loader=yaml.FullLoader)

		# Init database communication
		self.comm_local_consensus = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_local_consensus.sql_open()

	def main(self, _, stop):

		# Open tcp server
		print("TA-agent:	Main thread started")
		with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock_server:

			# Set socket
			sock_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
			sock_server.bind((self.params['ip'], self.params['port']))
			sock_server.listen()
			sock_server.settimeout(None)
			time.sleep(1)

			# Loop
			while True:

				# Listening for incomming messages
				print("\nTask allocation agent:      	        Listening for incoming tasks...")
				conn, _ = sock_server.accept()

				# Handle in separate thread
				start_new_thread(self.handle_client, (conn,))

				# Close thread at close event 
				if stop():
					print("TA-agent:	Main thread killed")
					break

	def handle_client(self, conn):

		# Open database connection
		comm = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		comm.sql_open()

		# Receive message
		data = conn.recv(2024)
							
		# Convert data to dictionary
		task = pickle.loads(data)
		print('\nAGV ' + str(self.params['id']) +'         Received task: ' + str(task['id']))

		# Compute bid or add directly to local task list
		if task['message'] == 'announce':

			# Get local task list
			local_task_list = comm.sql_get_local_task_list(self.params['id'])

			# Compute bid
			bid_value_list = self.compute_bid(local_task_list, task, False, comm)

			# Make robot dict
			robot = {'id': self.params['id'], 'ip': self.params['ip'], 'port': self.params['port']}

			# Make bid dict
			bid = {'values': bid_value_list, 'robot': robot, 'task': task} if bid_value_list is not None else None

			# Send bid to the auctioneer
			conn.sendall(pickle.dumps(bid))

		elif task['message'] == 'assign':

			# Add assigned tasks optimally to local task list
			task_dict = {'robot': self.params['id'], 'status': 'assigned', 'priority': 1, 'task_bids': str(task['task_bids']), 'message': 'assign'}
			comm.sql_update_task(task['id'], task_dict)

		# Close connection
		conn.close()
		comm.sql_close()

	def local_consensus(self, _, stop):

		# Loop
		print("TA-agent:	Local consensus thread started")
		while True:

			# Timeout
			time.sleep(self.params['local_consensus_rate'])

			# Get tasks to assign
			tasks_to_assign = self.comm_local_consensus.sql_get_local_task_list(self.params['id'])

			# Filter out homing and charging tasks
			tasks_to_assign = [task for task in tasks_to_assign if not task['message'] == 'homing' and not task['message'] == 'charging']

			# Get all robots in the fleet
			robots = self.get_all_robots_exept_this(self.comm_local_consensus)
			
			# Make assignment with all tasks in task list
			if tasks_to_assign is not None and robots is not None:
				if not len(tasks_to_assign) == 0 and not len(robots) == 0:

					# Create all announce messages
					combinations = []
					for robot in robots:
						for task in tasks_to_assign:
							task['message'] = "announce"
							combinations.append((robot['ip'], robot['port'], task))

					# Send all announcement messages and receive all bids
					self.p = Pool(processes=max(1, len(combinations)))
					bids = self.p.starmap(tcp_client, combinations)
					bids = [bid for bid in bids if bid is not None] # Remove all None bids = AGV not participating

					# If there are valid bids
					if len(bids) > 0: 
								
						# Resolution
						best_bid_list = self.resolution_lb(bids)
						best_bid = best_bid_list['values'][0] # self.params['epsilon'] * best_bid_list['values'][0] + (1 - self.params['epsilon']) * best_bid_list['values'][1]

						# My bid
						my_bid_list = self.compute_bid(tasks_to_assign, task, True, self.comm_local_consensus)

						# If I want to participate
						if my_bid_list is not None:
							my_bid = my_bid_list[0] # self.params['epsilon'] * my_bid_list[0] + (1 - self.params['epsilon']) * my_bid_list[1]
						else:
							my_bid = float('inf')

						# If best bid is better than my bid
						if best_bid + my_bid < 0:

							# Send task to winning robot
							print("Better solution found")
							robot = best_bid_list['robot']
							best_bid_list['task']['message'] = 'assign'
							best_bid_list['task']['robot'] = robot['id']
							tcp_client(robot['ip'], robot['port'], best_bid_list['task'])

			# Close thread at close event 
			if stop():
				print("TA-agent:	Local consensus thread killed")
				break

	def get_all_robots_exept_this(self, comm):
		items = comm.sql_select_everything_from_table('global_robot_list')
		if items is not None:
			items_ = [robot for robot in items if robot['id'] != self.params['id']]
		else:
			items_ = []
		return items_ 

	def resolution_lb(self, bids):
		bids.sort(key=lambda p: p['values'][0]) # self.params['epsilon'] * p['values'][0] + (1 - self.params['epsilon']) * p['values'][1])
		best_bid = bids[0] 
		return best_bid	
				
	def compute_bid(self, local_task_list, new_task, substract, comm):

		"""

		Computes the difference between the estimated current cost of the local task list and the 
		estimated cost of the local task list including the new task, as well as the estimated 
		start time, end time, and duration of the new task

		Input:
			- Local task list
			- New task
			- Substract the new task from the local task list or add it 
			- Communication layer
		Output:
			- Objective list

		"""

		# Assertions
		assert isinstance(local_task_list, list)
		assert isinstance(new_task, dict)
		assert isinstance(substract, bool)
		assert isinstance(comm, Comm)

		# Get robot
		robot = comm.sql_get_robot(self.params['id'])[0]

		# Get task executing
		task_executing = comm.sql_get_executing_task(self.params['id'])

		# Do not bid on task that is already in local task list
		if new_task['node'] in [task['node'] for task in local_task_list] or new_task['node'] == robot['node']: return None
		if not len(task_executing) == 0:
			if new_task['node'] == task_executing[0]['node']:
				return True
			
		# Remove homing node
		local_task_list = [task for task in local_task_list if not task['message'] == 'homing' and not task['message'] == 'charging']

		# Get new local task list
		new_local_task_list = local_task_list + [new_task] if not substract else [t for t in local_task_list if t['id'] != new_task['id']]

		# Start node
		start_node = robot['node'] if len(task_executing) == 0 else task_executing[0]['node']

		# Tsp + routing,for current local task list
		_, _, _, _, current_costs = self.tsp_routing(start_node, local_task_list, comm)
		
		# Tsp + routing fornewlocal task list
		new_sequence, _, _, _, new_costs = self.tsp_routing(start_node, new_local_task_list, comm)

		# Marginal cost to execute task
		min_sum = sum(new_costs) - sum(current_costs)
		min_max = sum(new_costs)

		# Compute start, end and duration
		if not substract:
			task_index = new_sequence.index(new_task)
			start_time = timedelta(seconds=sum(new_costs[0:task_index])) + datetime.now()
			end_time = timedelta(seconds=sum(new_costs[0:task_index+1])) + datetime.now()
			duration = end_time - start_time
		else:
			start_time = 0.0
			end_time = 0.0
			duration = 0.0

		# Objective list
		objective_list = [min_sum, min_max, start_time, end_time, duration]

		return objective_list

	def tsp_routing(self, start_node, new_local_task_list, comm):

		# Get graph
		self.graph = comm.get_graph()

		# Extract only node names from tasks without homing task
		nodes_to_visit = [task['node'] for task in new_local_task_list]

		# Do tsp using astar routing
		solution = tsp(start_node, nodes_to_visit, self.dist_astar)

		# Convert task node names back to tasks including homing task
		sequence = [new_local_task_list[nodes_to_visit.index(name)] for name in solution['sequence']]

		# Find best paths towards all tasks
		best_paths = []
		best_slots= []
		best_dists = []
		best_costs = []
		for task in sequence:

			# Get path distance
			path, dist = get_shortest_path(self.graph, start_node, task['node'])
			slots = [[0, 0]]
			cost = dist

			# Best route selection
			best_paths.append(path)
			best_slots.append(slots)
			best_dists.append(dist)
			best_costs.append(cost)

			# Update state
			start_node = task['node']

		return sequence, best_paths, best_slots, best_dists, best_costs

	def dist_astar(self, a, b):
		return get_shortest_path(self.graph, a, b)


def tcp_client(ip, port, data):

	try:

		# Create a TCP/IP socket
		with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:

			#Set socket timeout
			sock.settimeout(10)

			# Make message
			pickled_data = pickle.dumps(data)

			# Connect the socket to the port where the server is listening
			sock.connect((ip, port))
								
			# Send data
			sock.sendall(pickled_data)

			# Look for the response
			data = sock.recv(2024)
			rec_data_loaded = pickle.loads(data)
			return rec_data_loaded

	except:
		print("Central auctioneer:		Received nothing")
		return None	