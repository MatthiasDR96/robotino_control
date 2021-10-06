import matplotlib.pyplot as plt
from datetime import datetime
from robotino_core.agv.AGV_Main import AGV
from robotino_core.Comm import Comm
from robotino_core.agv.AGV_RO_agent import RO_agent

# Params
robot_id = 15
start_node = 'pos_1'
nodes_to_visit = ['pos_10']
start_time = datetime.now()

# AGV
agv = AGV('localhost', 10015, 15, 'localhost', 'matthias', 'matthias', 'kb', (10, 30), False, False, False)

# Open connection to database
comm = Comm('localhost', 10015, 'localhost', 'matthias', 'matthias', 'kb')
comm.sql_open()

# Calculate estimated end time and duration
routing = RO_agent(agv)

# Get start node of robot
start_node = 'pos_1'

# Get start time of robot
start_time = datetime.now() 

# Get tasks in local task list
local_task_list =  ['pos_10']

# Task executing
task_executing = ['pos_6']

# Nodes to visit
nodes_to_visit = task_executing + local_task_list

# Update paths for each task in local task list
reserved_paths = {}
reserved_slots  = {}
for task in nodes_to_visit:

	# Do dmas
	best_path, best_slots, best_dist, best_cost = routing.dmas(start_node, task, start_time, comm)
	
	# Set paths towards all tasks
	reserved_paths[task] = best_path
	reserved_slots[task] = best_slots

	# Update state
	start_node = task
	start_time = best_slots[-1][0] + best_slots[-1][1]

# Print
print(reserved_paths)
print(reserved_slots)

# Plot
ax = plt.subplot()
agv.graph.plot(ax)
plt.show()


