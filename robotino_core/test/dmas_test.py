from datetime import datetime
from robotino_core.agv.AGV_Main import AGV
from robotino_core.Comm import Comm
from robotino_core.agv.AGV_Routing import Routing

# Params
robot_id = 15
start_node = 'pos_1'
nodes_to_visit = ['pos_10']
start_time = datetime.now()

# AGV
agv = AGV('localhost', 10015, 15, 'localhost', 'matthias', 'matthias', 'kb', (10, 30))

# Open connection to database
comm = Comm('localhost', 10015, 'localhost', 'matthias', 'matthias', 'kb')
comm.sql_open()

# Calculate estimated end time and duration
routing = Routing(agv)

# Get start node of robot
start_node = 'pos_1'

# Get start time of robot
start_time = datetime.now() 

# Get tasks in local task list
local_task_list =  ['pos_10']

# Get executing task
task_executing =  ['pos_6']

# Nodes to visit
nodes_to_visit = task_executing + local_task_list

# Update paths for each task in local task list
reserved_paths = {}
reserved_slots  = {}
total_path = []
estimated_start_time = start_time
for task in nodes_to_visit:

	# Do dmas
	best_path, best_slots = routing.dmas(start_node, [task], estimated_start_time, comm)

	# If a solution exist
	if best_path:

		# Set total planned path
		total_path.extend(best_path)

		# Estimated end time and duration
		estimated_end_time = best_slots[-1][0] + best_slots[-1][1]
		estimated_duration = estimated_end_time - estimated_start_time

		# Reserve slots
		routing.intent(best_path, best_slots, comm)

		# Set paths towards all tasks
		reserved_paths[task] = best_path
		reserved_slots[task] = best_slots
		
		# Estimated start time for next task
		estimated_start_time = estimated_end_time

		# Start node for next task is end node of previous task
		start_node = task

	agv.path = total_path

print(reserved_paths)
print(reserved_slots)


