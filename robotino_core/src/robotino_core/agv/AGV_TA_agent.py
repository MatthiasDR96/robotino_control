from datetime import datetime, timedelta
import time
import pickle
import socket
from multiprocessing import Pool
from _thread import start_new_thread

from robotino_core.Comm import Comm

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
						best_bid = best_bid_list['values'][0] # self.agv.params['epsilon'] * best_bid_list['values'][0] + (1 - self.agv.params['epsilon']) * best_bid_list['values'][1]

						# My bid
						my_bid_list = self.compute_bid(tasks_to_assign, task, True, self.comm_local_consensus)

						# If I want to participate
						if my_bid_list is not None:
							my_bid = my_bid_list[0] # self.agv.params['epsilon'] * my_bid_list[0] + (1 - self.agv.params['epsilon']) * my_bid_list[1]
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
			items_ = [robot for robot in items if robot['id'] != self.agv.id]
		else:
			items_ = []
		return items_ 

	def resolution_lb(self, bids):
		bids.sort(key=lambda p: p['values'][0]) # self.agv.params['epsilon'] * p['values'][0] + (1 - self.agv.params['epsilon']) * p['values'][1])
		best_bid = bids[0] 
		return best_bid	
				
	def handle_client(self, conn):

		# Open database connection
		comm = Comm(self.agv.ip, self.agv.port, self.agv.host, self.agv.user, self.agv.password, self.agv.database)
		comm.sql_open()

		# Receive message
		data = conn.recv(2024)
							
		# Convert data to dictionary
		task = pickle.loads(data)
		print('\nAGV ' + str(self.agv.id) +'         Received task: ' + str(task['id']))

		# Compute bid or add directly to local task list
		if task['message'] == 'announce':

			# Get local task list
			local_task_list = comm.sql_get_local_task_list(self.agv.id)

			# Compute bid
			bid_value_list = self.compute_bid(local_task_list, task, False, comm)

			# Make robot dict
			robot = {'id': self.agv.id, 'ip': self.agv.ip, 'port': self.agv.port}

			# Make bid dict
			bid = {'values': bid_value_list, 'robot': robot, 'task': task} if bid_value_list is not None else None

			# Send bid to the auctioneer
			conn.sendall(pickle.dumps(bid))

			# Print 
			if bid is None:
				print("Agv " + str(self.agv.id) + ":        Sent bid " + str(bid) + " to auctioneer")
			else:
				print("Agv " + str(self.agv.id) + ":        Sent bid " + str(bid['values'][:2]) + " to auctioneer")			

		elif task['message'] == 'assign':

			# Add assigned tasks optimally to local task list
			task_dict = {'robot': self.agv.id, 'status': 'assigned', 'priority': 1, 'task_bids': str(task['task_bids']), 'message': 'assign'}
			result = comm.sql_update_task(task['id'], task_dict)

			# Print
			if result:
				conn.sendall(pickle.dumps('task_accepted'))
				print("Agv " + str(self.agv.id) + ":        Added task " + str(task['id']) + " to local task list")
			else:
				conn.sendall(pickle.dumps('ERROR'))
				print("Agv " + str(self.agv.id) + ":        Could not add task to local task list")
	
		# Close connection
		conn.close()
		comm.sql_close()

	def compute_bid(self, local_task_list, task, substract, comm):

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
		assert isinstance(task, dict)
		assert isinstance(substract, bool)

		# Do not bid on task that is already in local task list
		if task['node'] in [task['node'] for task in local_task_list] or task['node'] == self.agv.node or task['node'] == self.agv.task_executing['node']:
			return None

		# Start situation
		start_time = self.agv.task_executing_estimated_end_time if not self.agv.task_executing['id'] == -1 else datetime.now()
		start_node = self.agv.task_executing['node'] if not self.agv.task_executing['id'] == -1 else self.agv.node

		# Get new local task list
		new_local_task_list = local_task_list + [task] if not substract else [t for t in local_task_list if t['id'] != task['id']]

		# TSP wrapper
		sequence, _, _, _, costs = self.agv.tsp_dmas(start_node, start_time, new_local_task_list, comm)

		# Calculate new cost
		new_cost = sum(costs)

		# Marginal cost to execute task
		min_sum = new_cost - sum(self.agv.local_task_list_estimated_cost)
		min_max = new_cost

		# Compute start, end and duration
		if not substract:
			task_index = sequence.index(task)
			start_time = timedelta(seconds=sum(costs[0:task_index])) + datetime.now()
			end_time = timedelta(seconds=sum(costs[0:task_index+1])) + datetime.now()
			duration = end_time - start_time
		else:
			start_time = 0.0
			end_time = 0.0
			duration = 0.0

		# Objective list
		objective_list = [min_sum, min_max, start_time, end_time, duration]

		return objective_list

def tcp_client(ip, port, data):

	try:

		# Create a TCP/IP socket
		with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
			sock.settimeout(10)

			# Make message
			pickled_data = pickle.dumps(data)

			# Connect the socket to the port where the server is listening
			print("AGV	Announces task " + str(data['id']) + " to AGV " + str(port) + ' at ' + str(datetime.now().time()))
			sock.connect((ip, port))
								
			# Send data
			sock.sendall(pickled_data)

			# Look for the response
			data = sock.recv(2024)
			rec_data_loaded = pickle.loads(data)
			if not isinstance(rec_data_loaded, str):
				print("AGV	Received " + str(rec_data_loaded['values'][:2]) + " from AGV " + str(rec_data_loaded['robot']['id']) + ' on task ' + str(rec_data_loaded['task']['id']) + ' at ' + str(datetime.now().time()))
			else:
				print("Central auctioneer:		Received " + rec_data_loaded)
			return rec_data_loaded

	except:
		print("Central auctioneer:		Received nothing")
		return None	