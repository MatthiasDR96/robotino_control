import threading
import time
import pickle
import socket
import os
import yaml
import ast

from robotino_core.Comm import Comm
from robotino_core.solvers.tsp_solver import *
from robotino_core.solvers.astar_solver import *

class AGV_TA_agent:

	"""
			A class containing the intelligence of the Task Allocation agent. This agent takes care of
			task collection from an external MES system, task allocation of new task among robots, 
			and task switching of assinged tasks to improve the solution.
	"""

	def __init__(self, params_file):

		# Read params
		this_dir = os.path.dirname(os.path.dirname(__file__))
		data_path = os.path.join(this_dir, "params", params_file)
		with open(data_path, 'r') as file:
			self.params = yaml.load(file, Loader=yaml.FullLoader)

		# Init database communication for all threads
		self.comm_task_collector = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_task_allocator = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_task_switcher = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])
		self.comm_task_scheduler = Comm(self.params['ip'], self.params['port'], self.params['host'], self.params['user'], self.params['password'], self.params['database'])

		# Exit event
		self.exit_event = threading.Event()

		# Processes
		self.threads = []
		self.threads.append(threading.Thread(target=self.task_collector))
		self.threads.append(threading.Thread(target=self.task_allocator))
		self.threads.append(threading.Thread(target=self.task_switcher))
		self.threads.append(threading.Thread(target=self.task_scheduler))

	def signal_handler(self, _, __):
		print("\n\nShutdown TA-agent")
		self.exit_event.set()

	def task_collector(self):

		"""
		This thread continuously listens to incoming tasks from an external system at its IP-address. 
		If a task arrives, it puts it unassigned onto the global task list.

		"""

		# Open tcp server
		print("\nTA-agent:	Task collector thread started")
		with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock_server:
			sock_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
			sock_server.bind((self.params['ip'], self.params['port']))
			sock_server.listen()
			sock_server.settimeout(None)
			time.sleep(1)

			# Loop
			while True:

				# Listening for incomming messages
				print("\nTA-agent:	Listening for incoming tasks...")
				conn, _ = sock_server.accept()

				# Receive the data and retransmit it
				with conn:
					data = conn.recv(1024)
					conn.sendall(data)
								
				# Convert data to dictionary
				try:
					data = pickle.loads(data)
				except EOFError:
					data = None
				print("\nTA-agent:	Received task " + str(data))

				# Get all tasks
				tasks = self.comm_task_collector.sql_select_everything_from_table('global_task_list')
				if tasks is None: continue

				# Put unnasigned task on global task list if not present yet
				print(str([task for task in tasks if task["order_number"] == data['order_number']]))
				if len([task for task in tasks if task["order_number"] == data['order_number']]) == 0:
					task_dict = {"order_number": data['order_number'], "node": data['node'], "priority": 0, "robot": -1, "estimated_start_time": "-", "estimated_end_time": "-", "estimated_duration": "-", "real_start_time": "-", "real_end_time": "-", "real_duration": "-", "progress": 0.0, "message": "-", "status": 'unassigned', "approach": data["approach"], "auctioneer": -1, "task_bids": '{}',"path": '[]', "slots": "[]", "dist": 0.0, "cost": 0.0}
					self.comm_task_collector.sql_add_to_table('global_task_list', task_dict)

	def task_allocator(self):

		"""
		This thread continuously reviews all tasks in the global_task_list that are unassigned and places a bid on this task. 
		If it sees that after placing the bid, the number of placed bids equals the number of robots in the system, it selects 
		the robot with the best bid and allocates the task to this robot. It does this at a certain 'task_allocator_rate'.

		"""

		# Loop
		print("TA-agent:	Task allocator thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=self.params['task_allocator_rate']): 
				print("\nTA-agent:	Task allocator thread killed")
				break

			# Get unnasigned tasks
			unassigned_tasks = self.comm_task_allocator.sql_select_unassigned_tasks()
			if unassigned_tasks is None or len(unassigned_tasks) == 0: continue

			# Get all robots in the fleet
			robots = self.comm_task_allocator.sql_select_everything_from_table('global_robot_list')
			if robots is None or len(robots) == 0: continue

			# Get local task list
			local_task_list = self.comm_task_allocator.sql_select_local_task_list(self.params['id'])
			if local_task_list is None: continue

			# Get unassigned tasks not yet bid on
			unassigned_tasks = [task for task in unassigned_tasks if self.params['id'] not in ast.literal_eval(task['task_bids']).keys()]
			if len(unassigned_tasks) == 0: continue 

			# Get first unassinged task and convert taks to dictionary
			task = unassigned_tasks[0]
			task_bids = ast.literal_eval(task['task_bids'])

			# Compute bid list and select bid regarding objective index
			bid_value_list = self.compute_bid(local_task_list, task, False, self.comm_task_allocator)
			bid = 1000 if not bid_value_list else bid_value_list[self.params['objective_index']]

			# Place bid at task
			task_bids[self.params['id']] = bid
			task_dict = {'task_bids': str(task_bids)}
			self.comm_task_allocator.sql_update_task(task['id'], task_dict)

			# Allocate task
			if len(task_bids.keys()) >= len(robots):
				winner_id = min(task_bids, key=task_bids.get)
				task_dict = {'robot': winner_id, 'status': 'assigned'}
				self.comm_task_allocator.sql_update_task(task['id'], task_dict)			

	def task_switcher(self):

		"""
		This thread continuously reviews all tasks in the global_task_list that are assigned to robots and looks 
		if a better solution could be found. It does this at a certain 'task_swither_rate'.

		"""

		# Loop
		print("TA-agent:	Task switcher thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=self.params['task_switcher_rate']): 
				print("TA-agent:	Task switcher thread killed")
				break

			# Get asigned tasks
			assigned_tasks = self.comm_task_switcher.sql_select_assigned_tasks()
			if assigned_tasks is None or len(assigned_tasks) == 0: continue

			# Get local task list
			local_task_list = self.comm_task_switcher.sql_select_local_task_list(self.params['id'])
			if local_task_list is None: continue
			
			# Pick random task and convert task to dictionary
			task = assigned_tasks[random.randint(0, len(assigned_tasks)-1)]
			task_bids = ast.literal_eval(task['task_bids'])
				
			# Compute bid different if it is my own task compared to if it is another robot's task
			subtract = True if task['robot'] == self.params['id'] else False
			bid_multiplier = -1 if task['robot'] == self.params['id'] else 1

			# Compute bid list and select bid regarding objective index
			bid_value_list = self.compute_bid(local_task_list, task, subtract, self.comm_task_switcher)
			bid = 1000 if not bid_value_list else bid_value_list[self.params['objective_index']]

			# Place bid at task
			task_bids[self.params['id']] = bid * bid_multiplier
			task_dict = {'task_bids': str(task_bids)}
			self.comm_task_switcher.sql_update_task(task['id'], task_dict)

			# Allocate task
			winner_id = min(task_bids, key=task_bids.get)
			task_dict = {'robot': winner_id}
			self.comm_task_switcher.sql_update_task(task['id'], task_dict)	

	def task_scheduler(self):	

		"""
		This thread continuously updates the order of the local task list. It does this at a certain 'task_scheduler_rate'.

		"""

		# Loop
		print("TA-agent:	Task scheduler thread started")
		while True:

			# Timeout and exit event
			if self.exit_event.wait(timeout=self.params['task_scheduler_rate']): 
				print("TA-agent:	Task scheduler thread killed")
				break

			# Get graph
			self.graph = self.comm_task_scheduler.sql_select_graph()
			if self.graph is None: return None

			# Get robot
			robot = self.comm_task_scheduler.sql_select_robot(self.params['id'])
			if robot is None or len(robot) == 0: return None

			# Get task executing
			task_executing = self.comm_task_scheduler.sql_select_executing_task(self.params['id'])
			if task_executing is None: return None

			# Get local task list
			local_task_list = self.comm_task_scheduler.sql_select_local_task_list(self.params['id'])
			if local_task_list is None: continue

			# Get start node and new local task list
			start_node = robot[0]['node'] if len(task_executing) == 0 else task_executing[0]['node']

			# Execute tsp using astar routing and only node names of the tasks
			nodes_to_visit = [task['node'] for task in local_task_list]
			solution = tsp(start_node, nodes_to_visit, self.dist_astar)

			# Convert task node names back to tasks including homing task
			sequence = [local_task_list[nodes_to_visit.index(name)] for name in solution['sequence']] 

			# Update priorities
			for priority, task in enumerate(sequence):
				task_dict = {'priority': priority + 1}
				self.comm_task_scheduler.sql_update_task(task['id'], task_dict)		

				
	def compute_bid(self, local_task_list, new_task, substract, comm):

		"""

		Computes the difference between the estimated current cost of the local task list and the 
		estimated cost of the local task list including/excluding the new task. The estimated current 
		cost is calculated by a performance monitor thread in the main vehicle agent.

		Input:
			- Local task list
			- New task
			- Substract the new task from the local task list or add it 
			- Communication layer object
		Output:
			- Objective list

		"""

		# Assertions
		assert isinstance(local_task_list, list)
		assert isinstance(new_task, dict)
		assert isinstance(substract, bool)
		assert isinstance(comm, Comm)

		# Get graph
		self.graph = comm.sql_select_graph()
		if self.graph is None: return None

		# Get robot
		robot = comm.sql_select_robot(self.params['id'])
		if robot is None or len(robot) == 0 : return None

		# Get task executing
		task_executing = comm.sql_select_executing_task(self.params['id'])
		if task_executing is None: return None

		# Get start node and new local task list
		start_node = robot[0]['node'] if len(task_executing) == 0 else task_executing[0]['node']
		new_local_task_list = local_task_list + [new_task] if not substract else [t for t in local_task_list if t['id'] != new_task['id']]

		# Check if task occurs twice in list of all nodes
		all_nodes = [task['node'] for task in new_local_task_list] + [start_node]
		if not len(set(all_nodes)) == len(all_nodes): return None

		# Execute tsp using astar routing and only node names of the tasks
		nodes_to_visit = [task['node'] for task in new_local_task_list]
		solution = tsp(start_node, nodes_to_visit, self.dist_astar)

		# TODO: Insert optimal charging station (remove charging station, and add new if necessary)

		# TODO: Compute dmas routes between stations

		# Compute miniSum and miniMax objective values using the new and current costs of the local task list
		min_sum = sum(solution['dists']) - robot[0]['local_task_list_cost']
		min_max = sum(solution['dists'])

		# Compute objective list
		objective = self.params['epsilon'] * min_sum + (1 - self.params['epsilon']) * min_max
		objective_list = [objective, min_sum, min_max]

		return objective_list

	def dist_astar(self, a, b):
		return get_shortest_path(self.graph, a, b)