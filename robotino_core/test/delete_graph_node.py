import ast
from robotino_core.Comm import Comm

# Node to delete
node_to_delete = 'pos_13'

# Open connection to database
comm = Comm('127.0.0.1', 10015, '127.0.0.1', 'root', 'abc123', 'kb')

# Get graph
graph = comm.sql_select_everything_from_table('graph')

# Delte node
comm.sql_delete_from_table('graph', 'name', node_to_delete)

# Delete neighbors
for item in graph:
    if node_to_delete in item['neighbors']:
        new_neighbors = ast.literal_eval(item['neighbors'])
        new_neighbors.remove(node_to_delete)
        comm.sql_update_graph(item['name'], str(new_neighbors))


