from robotino_factory.Comm import Comm
from datetime import date, datetime, timedelta

def check_slot(node, slot):

	# Assertions
	assert isinstance(node, str)
	assert isinstance(slot[0], datetime)
	assert isinstance(slot[1], timedelta)

	# Check all available slots of other robots
	available_slots = get_available_slots(node, slot)

	# Take first available slot and adapt the end time to the required end time
	first_available_slot = available_slots[0]
	first_available_slot = (first_available_slot[0], slot[1])

	# Compute delay
	delay = first_available_slot[0] - slot[0]
	return first_available_slot, delay

def get_available_slots(node, slot, robot):

	# Assertions
	assert isinstance(node, str)
	assert isinstance(slot[0], datetime)
	assert isinstance(slot[1], timedelta)

	# Get slot information
	reservation_time = slot[0]
	print(reservation_time)
	duration = slot[1]

	# Get all reservations
	reservations = comm.sql_select_from_table('environmental_agents', 'node', node)

	# Get all reservations from the requested reservation_time for all other robots
	slots = []
	for res in reservations:
		start_time = datetime.strptime(res['start_time'], '%Y-%m-%d %H:%M:%S')
		end_time = datetime.strptime(res['end_time'], '%Y-%m-%d %H:%M:%S')
		if end_time > reservation_time and not res['robot'] == robot:
			slots.append((start_time, end_time))

	# Get free slots
	free_slots = []
	if slots:
		# Free slot from requested reservation time till start time of eariest reservation
		if slots[0][0] - reservation_time >= duration:
			free_slots.append((reservation_time, slots[0][0]))
		# Free intermediate slots
		for start, end in ((slots[i][1], slots[i + 1][0]) for i in range(len(slots) - 1)):
			if end - start >= duration:
				free_slots.append((start, end - start))
		# Free slot from last reservation time till infinity
		free_slots.append((slots[-1][1], float('inf')))
	else:
		free_slots.append((reservation_time, float('inf')))
	return free_slots

def reserve_slot(node, slot, robot):

	# Assertions
	assert isinstance(node, str)
	assert isinstance(slot[0], datetime)
	assert isinstance(slot[1], timedelta)

	# Get slot
	reservation_start_time = slot[0]
	reservation_end_time = reservation_start_time + slot[1]

	# Get all reservations
	reservations = comm.sql_select_from_table('environmental_agents', 'node', node)

	valid = True
	for res in reservations:
		start_time = datetime.strptime(res['start_time'], '%Y-%m-%d %H:%M:%S')
		end_time = datetime.strptime(res['end_time'], '%Y-%m-%d %H:%M:%S')
		if not res['robot'] == robot:
			if reservation_start_time <= start_time < reservation_end_time or reservation_start_time < end_time <= reservation_end_time:
				valid = False
				break
	if valid:
		reservation_dict = {'node': node, 'robot': robot, 'start_time': reservation_start_time.strftime('%Y-%m-%d %H:%M:%S'), 'end_time': reservation_end_time.strftime('%Y-%m-%d %H:%M:%S'), 'pheromone': 0.0}
		comm.sql_add_to_table('environmental_agents', reservation_dict)
		print("Slot (" + str(reservation_start_time) + ', ' + str(slot[1]) + ") reserved for agv " + str(robot))
	else:
		print("Could not add slot (" + str(reservation_start_time) + ', ' + str(reservation_end_time) + ") of agv " + str(robot))

def remove_reservations(agv_id):
	comm.sql_delete_from_table('environmental_agents', 'robot', agv_id)


if __name__ == "__main__":
	
	# Open connection to database
	comm = Comm('localhost', 10015, 'localhost', 'matthias', 'matthias', 'kb')
	comm.sql_open()
			
	# Make reservation
	node = 'pos_7'

	# Reserve slot
	wanted_slot = (datetime.now(), timedelta(seconds=20))
	reserve_slot(node, wanted_slot, 1)

	# Reserve slot
	wanted_slot = (datetime.now() + timedelta(seconds=10), timedelta(seconds=20))
	reserve_slot(node, wanted_slot, 2)

	# Check slots
	print(datetime.now())
	wanted_slot = (datetime.now() + timedelta(seconds=10), timedelta(seconds=10))
	slots = get_available_slots(node, wanted_slot, 3)
	print(slots)

	comm.sql_show_table('global_robot_list')
	# Remove reservations
	remove_reservations(1)

	# Close connection
	comm.sql_close()		
