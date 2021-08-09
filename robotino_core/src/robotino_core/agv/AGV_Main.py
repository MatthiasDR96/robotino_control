from datetime import datetime, timedelta
import threading
import yaml
import os
import time
import math
import signal

from robotino_core.Comm import Comm
from robotino_core.agv.AGV_Action import Action
from robotino_core.agv.AGV_RO_agent import RO_agent
from robotino_core.agv.AGV_TA_agent import TA_agent
from robotino_core.agv.AGV_RM_agent import RM_agent
from robotino_core.solvers.astar_solver import find_shortest_path

class AGV:

	"""
		A class containing the intelligence of the agv agent
	"""

	def __init__(self, ip, port, id, host, user, password, database, loc, ta_agent, ro_agent, rm_agent):

		# Params
		self.ip = ip
		self.port = port
		self.host = host
		self.user = user
		self.password = password
		self.database = database

		# Read params
		this_dir = os.path.dirname(os.path.dirname(__file__))
		data_path = os.path.join(this_dir, "params", "setup.yaml")
		with open(data_path, 'r') as file:
			self.params = yaml.load(file, Loader=yaml.FullLoader)

		# Init database communication
		self.comm_main = Comm(self.ip, self.port, self.host, self.user, self.password, self.database)
		self.comm_main.sql_open()
		self.comm_odom_callback = Comm(self.ip, self.port, self.host, self.user, self.password, self.database)
		self.comm_odom_callback.sql_open()
		self.comm_cost_estimator = Comm(self.ip, self.port, self.host, self.user, self.password, self.database)
		self.comm_cost_estimator.sql_open()

		# Get graph
		self.graph = self.comm_main.get_graph()
		self.depot = self.search_closest_node(loc)
		
		# Current attributes - status at the moment
		self.id = id
		self.x_loc = loc[0]
		self.y_loc = loc[1]
		self.theta = 0.0
		self.node = self.search_closest_node(loc)
		self.status = 'IDLE'
		self.battery_status = 100.0

		# History attributes - work done
		self.start_time = datetime.now()
		self.traveled_cost = 0.0
		self.charged_time = 0.0
		self.congestions = 0
		self.delays = 0.0

		# Future attributes - work to be done
		self.local_task_list_estimated_cost = 0.0
		self.local_task_list_estimated_end_time = 0.0

		# Routing attributes
		self.reserved_paths = {} # All planned paths for tasks in local task list
		self.reserved_slots = {} # All reserved slots for tasks in local task list
		self.total_path = [] # Total planned path for tasks in local task list

		# Current executing task attributes
		self.task_executing = {'id': -1, 'node': None, 'message': '-'}
		self.task_executing_estimated_cost = 0.0
		self.task_executing_start_time = datetime.now() # Start time of executing task
		self.task_executing_estimated_end_time = datetime.now() # Estimated end time of executing task
		self.current_node = self.node # Current node moving to
		self.current_path = [] # Current path following

		# Task agents
		self.ta_agent = None
		self.ro_agent = None
		self.rm_agent = None		
		self.action = Action(self)

		# Exit procedure
		signal.signal(signal.SIGINT, self.signal_handler)
		self.stop_threads = False

		# Main AGV agent processes
		threads = []
		threads.append(threading.Thread(target=self.main, args=(1, lambda: self.stop_threads)))
		threads.append(threading.Thread(target=self.odom_callback, args=(2, lambda: self.stop_threads)))
		threads.append(threading.Thread(target=self.cost_estimator, args=(3, lambda: self.stop_threads)))

		# Task allocation agent processes
		if ta_agent:
			self.ta_agent = TA_agent(self)
			threads.append(threading.Thread(target=self.ta_agent.main, args=(4, lambda: self.stop_threads)))
			threads.append(threading.Thread(target=self.ta_agent.local_consensus, args=(5, lambda: self.stop_threads)))

		# Routing agent processes
		if ro_agent:
			self.ro_agent = RO_agent(self)
			threads.append(threading.Thread(target=self.ro_agent.main, args=(6, lambda: self.stop_threads)))
			threads.append(threading.Thread(target=self.ro_agent.homing, args=(7, lambda: self.stop_threads)))

		# Resource management agent processes
		if rm_agent:
			self.rm_agent = RM_agent(self)
			threads.append(threading.Thread(target=self.rm_agent.main, args=(8, lambda: self.stop_threads)))
		
		# Start processes
		for thread in threads:
			thread.daemon = True
			thread.start()

		# Join processes
		for thread in threads:
			thread.join()

	def main(self, _, stop):

		# Loop
		print("\nAGV-agent:	Main thread started")
		while True:

			# Wait for a task on the local task list and pick first (lowest priority)
			for row in self.comm_main.sql_get_local_task_list(self.id): self.task_executing = row; break 

			# Execute task if exist
			if not self.task_executing['id'] == -1:
				
				# Start task
				print("\nAgv " + str(self.id) + ":        Start executing task " + str(self.task_executing['id']))
				if self.status != 'EMPTY': self.status = 'BUSY'

				# Remove from local task list and add task to executing list
				self.task_executing_start_time = datetime.now()
				task_dict = {'robot': self.id, 'status': 'executing', 'real_start_time': self.task_executing_start_time.strftime('%H:%M:%S')}
				self.comm_main.sql_update_task(self.task_executing['id'], task_dict)

				# Move to task
				result = self.execute_task(self.task_executing)

				# If task reached
				if result:

					# Pick task
					if not self.task_executing['message'] in ['homing', 'charging']: self.action.pick()

					# Task executed
					end_time = datetime.now()
					duration = (end_time - self.task_executing_start_time).total_seconds()
					task_dict = {'status': 'done', 'real_end_time': end_time.strftime('%H:%M:%S'), 'real_duration': str(duration)}
					self.comm_main.sql_update_task(self.task_executing['id'], task_dict)
					print("Agv " + str(self.id) + ":        Finished task " + str(self.task_executing['id']))
					self.task_executing = {'id': -1, 'node': None}
					
				# If task not reached
				else:

					# Make task unnasigned if not able to reach
					task_dict = {'status': 'unassigned', 'message': '-', 'priority': 0}
					self.comm_main.sql_update_task(self.task_executing['id'], task_dict)
					print("Agv " + str(self.id) + ":        Failed task " + str(self.task_executing['id']))
					self.task_executing = {'id': -1, 'node': None}
					
				# Set status to IDLE when task is done or when done charging
				if self.status != 'EMPTY': self.status = 'IDLE'

			# Close thread at close event 
			if stop():
				print("AGV-agent:	Main thread killed")
				break

	def execute_task(self, task):

		# Compute path towards task destination
		if self.ro_agent:
			if not task['id'] in self.reserved_paths.keys(): self.ro_agent.plan_executing_task(self.comm_main)
		else:
			path, _ = find_shortest_path(self.comm_main.get_graph(), self.node, task['node'])
			self.reserved_paths[task['id']] = path
			self.reserved_slots[task['id']] = [0]

		# Move from node to node until end node reached
		while not self.node == task['node']:

			# Set current path and current node moving to
			self.current_path = self.reserved_paths[task['id']]
			self.current_node = self.reserved_paths[task['id']][0]
			self.current_slot = self.reserved_slots[task['id']][0]

			# If routing agent
			if self.ro_agent:
				node_arriving_time = self.current_slot[0]
				if node_arriving_time > datetime.now():
					td = node_arriving_time - datetime.now()
					time.sleep(td.total_seconds())
					print("Agv " + str(self.id) + ":        Waits " + str(td) + " seconds")
			else:
				self.reserved_paths[task['id']] = self.reserved_paths[task['id']][1:]
					
			# Move to node if free		
			result = self.action.move_to_node(self.current_node)
			if not result: return False

		# Check if task is charging task
		if task['message'] == 'charging':

			# Update status
			print("AGV " + str(self.id) + ":        Is charging for " + str(5) + " seconds")
			self.status = 'CHARGING'

			# Charging
			time.sleep(5)

			# Update robot status
			self.battery_status = 100
			self.status = 'IDLE'

		return True

	def odom_callback(self, _, stop):

		# Loop
		print("AGV-agent:	Odom thread started")
		while True:

			# Make robot dict
			robot_dict = {"id": self.id, "ip": self.ip, "port": self.port, "x_loc": self.x_loc, "y_loc": self.y_loc, "theta": self.theta, "node": self.node,
				"status": self.status, "battery_status": self.battery_status, "traveled_cost": self.traveled_cost, "charged_time": self.charged_time,
				"congestions": self.congestions, "task_executing": self.task_executing['id'], "moving_to": self.current_node, "path": str(self.current_path), "total_path": str(self.total_path), "time_now": datetime.now().strftime('%H:%M:%S'),
				"executing_task_cost": self.task_executing_estimated_cost, "local_task_list_cost": self.local_task_list_estimated_cost, "local_task_list_end_time": self.local_task_list_estimated_end_time}

			# Update robot
			self.comm_odom_callback.sql_add_to_table('global_robot_list', robot_dict)
			self.comm_odom_callback.sql_update_robot(self.id, robot_dict)

			# Close thread at close event 
			if stop():
				print("AGV-agent:	Odom thread killed")
				break

	def cost_estimator(self, _, stop):

		# Loop
		print("AGV-agent:	Cost estimator thread started")
		while True:

			try:

				# Inputs
				current_time = datetime.now()
				current_location = (self.x_loc, self.y_loc)
				current_node = self.current_node
				current_task = self.task_executing if not self.task_executing['id'] == -1 else None
				current_local_task_list = self.comm_cost_estimator.sql_get_local_task_list(self.id)

				# If there are tasks on local task list
				if current_task is not None:
					
					if self.ro_agent:

						# Evaluate executing task
						end_time = self.reserved_slots[self.task_executing['id']][-1][0] + self.reserved_slots[self.task_executing['id']][-1][1]
						duration = (end_time - self.task_executing_start_time).total_seconds()
						dist_to_be_done = (self.task_executing_estimated_end_time - datetime.now()).total_seconds()
						self.traveled_cost = (datetime.now() - self.start_time).total_seconds()
						self.task_executing_estimated_cost = dist_to_be_done
						self.task_executing_estimated_end_time = end_time

						# Calculate progress
						estimated_cost = (self.task_executing_estimated_end_time - self.task_executing_start_time).total_seconds()
						
						progress = round(((estimated_cost - dist_to_be_done) / estimated_cost) * 100 if not estimated_cost == 0.0 else 0.0)

						# Update executing task
						task_dict = {'progress': progress, 'estimated_start_time': self.task_executing_start_time.strftime('%H:%M:%S'), 'estimated_end_time': end_time.strftime('%H:%M:%S'), 'estimated_duration': str(duration)}
						self.comm_cost_estimator.sql_update_task(current_task['id'], task_dict)

						# Evaluate local task list
						local_task_cost = 0.0
						for task in current_local_task_list:

							# Compute start, end and duration
							start_time = self.reserved_slots[task['id']][0][0]
							end_time = self.reserved_slots[task['id']][-1][0] + self.reserved_slots[task['id']][-1][1]
							duration = (end_time - start_time).total_seconds()

							# Update task
							task_dict = {'estimated_start_time': self.task_executing_start_time.strftime('%H:%M:%S'), 'estimated_end_time': end_time.strftime('%H:%M:%S'), 'estimated_duration': str(duration)}
							self.comm_cost_estimator.sql_update_task(task['id'], task_dict)
							local_task_cost += duration

					else:

						# Evaluate executing task
						dist1 = self.calculate_euclidean_distance(current_location, self.graph.nodes[current_node].pos)
						dist2 = find_shortest_path(self.graph, current_node, current_task['node'])[1]
						cost_to_executing_task = timedelta(seconds=(dist1+dist2)/self.params['robot_speed'])

						# End time
						dist_to_be_done = (self.task_executing_estimated_end_time - datetime.now()).total_seconds()
						end_time = current_time + cost_to_executing_task
						self.traveled_cost = (datetime.now() - self.start_time).total_seconds()
						self.task_executing_estimated_cost = dist_to_be_done
						self.task_executing_estimated_end_time = end_time

						# Duration
						duration = (end_time - self.task_executing_start_time).total_seconds()
					
						# Calculate progres
						progress = round(((duration - dist_to_be_done) / duration) * 100 if not duration == 0.0 else 0.0)

						# Update executing task
						task_dict = {'progress': progress, 'estimated_start_time': self.task_executing_start_time.strftime('%H:%M:%S'), 'estimated_end_time': end_time.strftime('%H:%M:%S'), 'estimated_duration': str(duration)}
						self.comm_cost_estimator.sql_update_task(current_task['id'], task_dict)

						# Evaluate local task list
						local_task_cost = 0.0
						start_time = end_time
						start_node = current_task['node']
						for task in current_local_task_list:

							# Compute start, end and duration
							dist = find_shortest_path(self.graph, start_node, task['node'])[1]
							cost_to_task = timedelta(seconds=dist/self.params['robot_speed'])
							end_time = start_time + cost_to_task
							duration = cost_to_task.total_seconds()

							# Update task
							task_dict = {'estimated_start_time': self.task_executing_start_time.strftime('%H:%M:%S'), 'estimated_end_time': end_time.strftime('%H:%M:%S'), 'estimated_duration': str(duration)}
							self.comm_cost_estimator.sql_update_task(task['id'], task_dict)
							local_task_cost += duration

							# New start
							start_time = end_time
							start_node = task['node']

					# Update
					self.local_task_list_estimated_cost = local_task_cost
					self.local_task_list_estimated_end_time = end_time

			except:
				print("Error")
				continue

			# Close thread at close event 
			if stop():
				print("AGV-agent:	Cost estimator thread killed")
				break				

	def evaluate_local_task_list(self, task_sequence, comm):

		# Start situation
		dist = self.calculate_euclidean_distance((self.x_loc, self.y_loc), self.graph.nodes[self.current_node].pos)
		dist += find_shortest_path(self.graph, self.current_node, self.task_executing['node'])[1] if not self.task_executing['id'] == -1 else 0.0
		start_time = datetime.now() + timedelta(seconds=dist/self.params['robot_speed'])
		start_node = self.task_executing['node'] if not self.task_executing['id'] == -1 else self.node

		# Loop
		travel_times = []
		if self.ro_agent:

			# Evaluate local task list
			for task in task_sequence:

				# Do dmas
				_, best_slots = self.ro_agent.dmas(start_node, [task['node']], start_time, comm)

				# Estimated end time
				end_time = best_slots[-1][0] + best_slots[-1][1]
				
				# Compute cost
				duration = end_time - start_time
				travel_times.append(float(duration.total_seconds()))

				# New start
				start_time = end_time
				start_node = task['node']
				
		else:

			# Evaluate local task list
			for task in task_sequence:

				# Do astar
				dist = find_shortest_path(self.graph, start_node, task['node'])[1]

				# Compute cost
				cost = timedelta(seconds=dist/self.params['robot_speed'])
				travel_times.append(float(cost.total_seconds()))

				# Estimated end time and duration
				end_time = start_time + cost
				duration = end_time - start_time
				
				# New start
				start_time = end_time
				start_node = task['node']

		return travel_times

	def search_closest_node(self, loc):
		graph = self.comm_main.get_graph()
		node = min(graph.nodes.values(), key=lambda node: self.calculate_euclidean_distance(node.pos, loc))
		return node.name

	@staticmethod
	def calculate_euclidean_distance(a, b):
		return math.sqrt(math.pow(b[0] - a[0], 2) + math.pow(b[1] - a[1], 2))

	def signal_handler(self, _, __):
		print("\n\nShutdown robot without communication possibilities")
		self.stop_threads = True
		time.sleep(1)
		exit(0)
