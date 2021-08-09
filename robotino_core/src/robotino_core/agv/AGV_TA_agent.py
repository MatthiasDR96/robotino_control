from datetime import datetime, timedelta
import time
import pickle
import socket
from multiprocessing import Pool
from _thread import start_new_thread

from robotino_core.Comm import Comm
from robotino_core.solvers.tsp_solver import tsp


class TA_agent:

	"""
			A class containing the intelligence of the Task Allocation agent. This agent takes care of
			including new announced tasks into the robots task list
	"""

	def __init__(self, agv):

		# AGV
		self.agv = agv

		# Init database communication
		self.comm_local_consensus = Comm(self.agv.ip, self.agv.port, self.agv.host, self.agv.user, self.agv.password, self.agv.database)
		self.comm_local_consensus.sql_open()

	def main(self, _, stop):

		# Open tcp server
		print("TA-agent:	Main thread started")
		with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock_server:
			sock_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
			sock_server.bind((self.agv.ip, self.agv.port))
			sock_server.listen()
			sock_server.settimeout(None)

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

	def local_consensus(self, _, stop):

		# Loop
		print("TA-agent:	Local consensus thread started")
		while True:

			# Timeout
			time.sleep(self.agv.params['local_consensus_rate'])

			# Get tasks to assign
			tasks_to_assign = self.comm_local_consensus.sql_get_local_task_list(self.agv.id)

			# Get all robots in the fleet
			robots = self.get_all_robots_exept_this(self.comm_local_consensus)
			
			# Make assignment with all tasks in task list
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
				bids = [bid for bid in bids if bid]

				# If there are valid bids
				if len(bids) > 0: 
							
					# Resolution
					best_bid_dict = self.resolution_lb(bids)
					best_bid = best_bid_dict['values'][0] # self.agv.params['epsilon'] * best_bid_list['values'][0] + (1 - self.agv.params['epsilon']) * best_bid_list['values'][1]

					# My bid
					my_bid_list = self.compute_bid(tasks_to_assign, task, self.comm_local_consensus, True) # datetime.strptime(task['estimated_start_time'], '%H:%M:%S').time() 

					# If I want to participate
					if my_bid_list is not None:
						my_bid = my_bid_list[0] # self.agv.params['epsilon'] * my_bid_list[0] + (1 - self.agv.params['epsilon']) * my_bid_list[1]
					else:
						my_bid = float('inf')

					# If best bid is better than my bid
					if best_bid + my_bid < 0:

						# Send task to winning robot
						print("Better solution found")
						robot = best_bid_dict['robot']
						best_bid_dict['task']['message'] = 'assign'
						best_bid_dict['task']['robot'] = robot['id']
						tcp_client(robot['ip'], robot['port'], best_bid_dict['task'])

			# Close thread at close event 
			if stop():
				print("TA-agent:	local consensus thread killed")
				break

	def get_tasks_to_assign(self, comm):
		items = comm.sql_select_from_table('global_task_list', 'status', "unassigned")
		items_ = [item for item in items if item['approach'] == 'decentral']
		return items_ if not items_ is None else []

	def get_all_robots_exept_this(self, comm):
		items = comm.sql_select_everything_from_table('global_robot_list')
		items_ = [robot for robot in items if robot['id'] != self.agv.id] if items is not None else []
		return items_ if not items_ is None else []

	def resolution_lb(self, bids):
		bids.sort(key=lambda p: self.agv.params['epsilon'] * p['values'][0] + (1 - self.agv.params['epsilon']) * p['values'][1])
		best_bid = bids[0] 
		return best_bid	
				
	def handle_client(self, conn):

		# Open database connection
		comm = Comm(self.agv.ip, self.agv.port, self.agv.host, self.agv.user, self.agv.password, self.agv.database)
		comm.sql_open()

		# Receive message
		data = conn.recv(1024)
						
		# Convert data to dictionary
		task = pickle.loads(data)
		print('\nAGV ' + str(self.agv.id) +'         Received task: ' + str(task['id']))

		# Get local task list
		local_task_list = comm.sql_get_local_task_list(self.agv.id)

		# Compute bid or add directly to local task list
		if task['message'] == 'announce':

			# Compute bid
			bid_value_list = self.compute_bid(local_task_list, task, comm, False)

			# Make bid structure
			if bid_value_list is not None:
				robot = {'id': self.agv.id, 'ip': self.agv.ip, 'port': self.agv.port}
				bid = {'values': bid_value_list, 'robot': robot, 'task': task}
			else:
				bid = None

			# Send bid to the auctioneer
			conn.sendall(pickle.dumps(bid))
			if bid is None:
				print("Agv " + str(self.agv.id) + ":        Sent bid " + str(bid) + " to auctioneer")
			else:
				print("Agv " + str(self.agv.id) + ":        Sent bid " + str(bid['values'][:2]) + " to auctioneer")			

		elif task['message'] == 'assign':

			# Add assigned tasks optimally to local task list
			updated_tasks = self.update_local_task_list(local_task_list, task)
			result = comm.sql_update_tasks(updated_tasks)
			if result:
				conn.sendall(pickle.dumps('task_accepted'))
				print("Agv " + str(self.agv.id) + ":        Added task " + str(task['id']) + " to local task list")

		# Close connection
		conn.close()
		comm.sql_close()

	def compute_bid(self, local_task_list, task, comm, substract):

		"""

		Computes the difference between the estimated current cost of the local task list and the 
		estimated cost of the local task list including the new task

		Input:
			- Local task list
			- New task
			- Substract the new task from the local task list or add it 
		Output:
			- Objective list

		"""

		# Assertions
		assert isinstance(local_task_list, list)
		assert isinstance(task, dict)
		assert isinstance(substract, bool)

		# Start node
		start_node = self.agv.task_executing['node'] if not self.agv.task_executing['id'] == -1 else self.agv.node

		# Get new local task list
		if not substract:
			new_local_task_list = local_task_list + [task]
		else:
			new_local_task_list = [t for t in local_task_list if t['id'] != task['id']]

		# Extract only node names from tasks
		nodes_to_visit = [task['node'] for task in new_local_task_list]

		# Optimize sequence
		task_sequence, _, _ = tsp(self.agv.graph, start_node, nodes_to_visit)

		# If one position occures multiple times, do not participate
		if not len(task_sequence) == len(new_local_task_list):
			return None

		# Convert task node names back to tasks
		task_sequence = [new_local_task_list[nodes_to_visit.index(name)] for name in task_sequence]

		# Consider resource management
		charging_time = 0.0
		if self.agv.rm_agent:
			charging_station, insertion_index, charging_time, _ = self.agv.resource_management.solve(task_sequence)
			if insertion_index is not None:
				task_dict = {"node": charging_station, "robot": self.agv.id, "message": 'charging', "status": 'assigned'}
				task_sequence.insert(insertion_index, task_dict)

		# Evaluate task sequence
		travel_times = self.agv.evaluate_local_task_list(task_sequence, comm)

		# Calculate new cost
		new_cost = sum(travel_times) + charging_time

		# Marginal cost to execute task
		min_sum = new_cost - self.agv.local_task_list_estimated_cost
		min_max = new_cost

		# Compute start, end and duration
		if not substract:
			task_index = task_sequence.index(task)
			start_time = timedelta(seconds=sum(travel_times[0:task_index])) + datetime.now()
			end_time = timedelta(seconds=sum(travel_times[0:task_index+1])) + datetime.now()
			duration = end_time - start_time
		else:
			start_time = 0.0
			end_time = 0.0
			duration = 0.0

		# Objective list
		objective_list = [min_sum, min_max, start_time, end_time, duration]

		return objective_list

	def update_local_task_list(self, local_task_list, task):

		"""

		Updates the local task list with the new task in an optimal sequence

		Input:
			- Local task list
			- New task
			- Start node
		Output:
			- Updates task list

		"""

		# Assertions
		assert isinstance(local_task_list, list)
		assert isinstance(task, dict)

		# Start node
		start_node = self.agv.task_executing['node'] if not self.agv.task_executing['id'] == -1 else self.agv.node

		# Get tasks in new local task list
		new_local_task_list = local_task_list + [task]

		# Extract only node names from tasks
		nodes_to_visit = [task['node'] for task in new_local_task_list]

		# Optimize sequence
		task_sequence, _, _ = tsp(self.agv.graph, start_node, nodes_to_visit)

		# Convert task node names back to tasks
		task_sequence = [new_local_task_list[nodes_to_visit.index(name)] for name in task_sequence]

		# Add new local task list
		tasks = []
		priority = 1
		for task in task_sequence:
			tasks.append((priority, self.agv.id, '-', '-', '-', '-', '-', '-', 0.0, 'assign', 'assigned', task['approach'], task['id']))
			priority += 1

		return tasks


def tcp_client(ip, port, data):

	try:

		# Create a TCP/IP socket
		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		sock.settimeout(5)

		# Make message
		pickled_data = pickle.dumps(data)

		# Connect the socket to the port where the server is listening
		print("Central auctioneer:		Announces task " + str(data['id']) + " to AGV " + str(port) + ' at ' + str(datetime.now().time()))
		sock.connect((ip, port))
						
		# Send data
		sock.sendall(pickled_data)

		# Look for the response
		rec_data = sock.recv(1024)
		rec_data_loaded = pickle.loads(rec_data)
		if not isinstance(rec_data_loaded, str):
			print("Central auctioneer:		Received " + str(rec_data_loaded['values'][:2]) + " from AGV " + str(rec_data_loaded['robot']['id']) + ' on task ' + str(rec_data_loaded['task']['id']) + ' at ' + str(datetime.now().time()))
		return rec_data_loaded

	except:
		return None
	finally:
		sock.close()	