from robotino_core.Comm import Comm
from datetime import datetime, timedelta

# Open connection to database
comm = Comm('127.0.0.1', 10015, '127.0.0.1', 'root', 'abc123', 'kb')
			
# Make reservation
node = 'pos_7'

# Get slot
slot = (datetime.now(), timedelta(seconds=20))
reservation_start_time = slot[0]
reservation_end_time = reservation_start_time + slot[1]

# Make reservation
reservation_dict = {'node': node, 'robot': 1, 'start_time': reservation_start_time.strftime('%Y-%m-%d %H:%M:%S'), 'end_time': reservation_end_time.strftime('%Y-%m-%d %H:%M:%S'), 'pheromone': 0.0}
comm.sql_add_to_table('environmental_agents', reservation_dict)
		

