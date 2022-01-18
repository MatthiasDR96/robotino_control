import ast
from robotino_core.Comm import Comm

# Node to add
node_to_add = 'pos_13'
node_neighbors = ['pos_8', 'pos_12', 'pos_16']

# Open connection to database
comm = Comm('127.0.0.1', 10015, '127.0.0.1', 'root', 'abc123', 'kb')

# Add node
table_dict = {'name': 'pos_13', 'x_loc': 60, 'y_loc': 90, 'theta': 0, 'neighbors': str(node_neighbors)}
comm.sql_add_to_table('graph', table_dict)

# Add neighbors
for neighbor in node_neighbors:
    tmp = comm.sql_select_from_table('graph', neighbor)
    print(tmp)
    tmp = ast.literal_eval(tmp[0]['neighbors'])
    tmp.append(node_to_add)
    print(tmp)
    comm.sql_update_graph(neighbor, str(tmp))