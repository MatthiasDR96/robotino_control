from datetime import datetime
from robotino_core.agv.AGV_Main import AGV
from robotino_core.Comm import Comm
from robotino_core.agv.AGV_Routing import Routing

# Params
id = 15
start_node = (10, 30)
nodes_to_visit = ['pos_10']
start_time = datetime.now()

# Start AGV
AGV('localhost', 10000+id, id, 'localhost', 'matthias', 'matthias', 'kb', start_node, True, False, False)