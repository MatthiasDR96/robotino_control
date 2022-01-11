from mysql.connector import connect, Error
import ast
import numpy as np
from operator import itemgetter
from robotino_core.Graph import Graph

class Comm:

	"""
		A library containing the interfaces to communicate with an SQL database. This library is the only interface to the 
		external infrastructure. So it suffices to only adapt this library for integration in other systems.
	"""

	def __init__(self, ip, port, host, user, password, database):

		# Params
		self.ip = ip
		self.port = port
		self.host = host
		self.user = user
		self.password = password
		self.database = database

		# Communication monitoring
		self.sql_queries = 0

		# Connect
		try:
			cnx = connect(
				host = self.host, 
				user = self.user, 
				password = self.password, 
				database = self.database
			)
			self.conn = cnx
			self.cursor = cnx.cursor(dictionary=True, buffered=True)
		except Error as e:
			print("Error %d: %s" % (e.args[0],e.args[1]))

	def sql_close(self):
		self.cursor.close()
		self.conn.close()			

	### Creat tables ### 

	def sql_create_robot_table(self, name):
		sql = """
		CREATE TABLE IF NOT EXISTS {}(
				id INT UNIQUE,
				ip VARCHAR(100),
				port INT,
				x_loc FLOAT,
				y_loc FLOAT,
				theta FLOAT,
				speed FLOAT,
				node VARCHAR(100),
				status VARCHAR(100),
				battery_status FLOAT,
				sql_queries INT,
				traveled_cost FLOAT,
				traveled_dist FLOAT,
				charged_time FLOAT,
				congestions INT,
				next_node VARCHAR(100),
				current_path LONGTEXT,
				total_path LONGTEXT,
				task_executing_dist FLOAT,
				task_executing_cost FLOAT,
				local_task_list_dist FLOAT,
				local_task_list_cost FLOAT,
				time_now VARCHAR(100)
		)
		""".format(name)
		self.cursor.execute(sql)
		self.conn.commit()

	### Drop tables

	def sql_drop_table(self, table):
		assert isinstance(table, str)
		sql = "DROP TABLE IF EXISTS {}".format(table)
		self.cursor.execute(sql)

	### Add to database ###

	def sql_add_to_table(self, table, item):
		assert isinstance(table, str)
		assert isinstance(item, dict)
		fields = ', '.join(item.keys())
		values = ', '.join(["%s"]*len(item.values()))
		sql = 'INSERT IGNORE INTO {} ({fields}) VALUES ({values})'.format(table, fields=fields, values=values)
		val = tuple(item.values())
		try:
			self.cursor.execute(sql, val)
			self.conn.commit()
			return self.cursor.lastrowid
		except:
			print("Database not alive1")
			return None

	### Select from database ###
		
	def sql_select_everything_from_table(self, table):
		assert isinstance(table, str)
		sql = "SELECT * FROM {}".format(table) + " ORDER BY id"
		try:
			self.conn.reconnect()
			self.cursor.execute(sql)
			result = self.cursor.fetchall()
			return result
		except:
			print("Database not alive1" + str(table))	
			return None

	def sql_select_robot_reservations(self, id):
		assert isinstance(id, int)
		sql = "SELECT * FROM environmental_agents WHERE robot = %s ORDER BY id"
		val = (id,)
		try:
			self.conn.reconnect()
			self.cursor.execute(sql, val)
			result = self.cursor.fetchall()
			return result
		except:
			print("Database not alive2")	
			return None

	def sql_select_reservations(self, table, node, id):
		assert isinstance(table, str)
		assert isinstance(node, str)
		assert isinstance(id, int)
		sql = "SELECT * FROM {} WHERE NOT robot = %s AND node = %s".format(table) + " ORDER BY id"
		val = (id, node)
		try:
			self.conn.reconnect()
			self.cursor.execute(sql, val)
			result = self.cursor.fetchall()
			return result
		except:
			print("Database not alive2")	
			return None

	def sql_select_robot(self, robot_id):
		assert isinstance(robot_id, int)
		sql = "SELECT * FROM global_robot_list WHERE id = %s"
		val = (robot_id,)
		try:
			self.conn.reconnect()
			self.cursor.execute(sql, val)
			result = self.cursor.fetchall()
			return result
		except:
			print("Database not alive3")	
			return None

	def sql_select_local_task_list(self, robot_id):
		assert isinstance(robot_id, int)
		sql = "SELECT * FROM global_task_list WHERE robot = %s AND status = 'assigned' ORDER BY priority"
		val = (robot_id,)
		try:
			self.conn.reconnect()
			self.cursor.execute(sql, val)
			result = self.cursor.fetchall()
			return result
		except:
			print("Database not alive4")	
			return None

	def sql_select_executing_task(self, robot_id):
		assert isinstance(robot_id, int)
		sql = "SELECT * FROM global_task_list WHERE robot = %s AND status = 'executing' ORDER BY priority"
		val = (robot_id,)
		try:
			self.conn.reconnect()
			self.cursor.execute(sql, val)
			result = self.cursor.fetchall()
			return result
		except:
			print("Database not alive5")	
			return None

	def sql_select_assigned_tasks(self):
		sql = "SELECT * FROM global_task_list WHERE status = 'assigned' AND NOT message = 'homing' AND NOT message = 'charging' "
		try:
			self.conn.reconnect()
			self.cursor.execute(sql)
			result = self.cursor.fetchall()
			return result
		except:
			print("Database not alive6")	
			return None

	def sql_select_unassigned_tasks(self):
		sql = "SELECT * FROM global_task_list WHERE status = 'unassigned' "
		try:
			self.conn.reconnect()
			self.cursor.execute(sql)
			result = self.cursor.fetchall()
			return result
		except:
			print("Database not alive7")	
			return None

	def sql_select_tasks_to_auction(self, robot_id):
		assert isinstance(robot_id, int)
		sql = "SELECT * FROM global_task_list WHERE auctioneer = %s AND status = 'unassigned' ORDER BY priority"
		val = (robot_id,)
		try:
			self.conn.reconnect()
			self.cursor.execute(sql, val)
			result = self.cursor.fetchall()
			return result
		except:
			print("Database not alive8")	
			return None

	def sql_select_graph(self):
		try:
			items = self.sql_select_everything_from_table('graph')
			graph = Graph()
			node_names = list(map(itemgetter('name'), items))
			node_locations = np.hstack((np.c_[list(map(itemgetter('x_loc'), items))], np.c_[list(map(itemgetter('y_loc'), items))], np.c_[list(map(itemgetter('theta'), items))]))
			node_neighbors = [ast.literal_eval(x) for x in list(map(itemgetter('neighbors'), items))]
			graph.create_nodes(node_locations, node_names)
			graph.create_edges(node_names, node_neighbors)
			return graph
		except:
			print("Database not alive9")	
			return None

	### Delete from database ###

	def sql_delete_from_table(self, table, criterium, value):
		assert isinstance(table, str)
		assert isinstance(criterium, str)
		sql = "DELETE FROM {} WHERE {} = %s".format(table, criterium)
		val = (value,)
		try:
			self.cursor.execute(sql, val)
			self.conn.commit()
			return True
		except:
			print("Database not alive10")	
			return False

	def sql_delete_everything_from_table(self, table):
		assert isinstance(table, str)
		assert isinstance(table, str)
		sql = "DELETE FROM {}".format(table)
		try:
			self.cursor.execute(sql)
			self.conn.commit()
			return True
		except:
			print("Database not alive11")	
			return False

	def sql_delete_local_task_list(self, robot_id):
		assert isinstance(robot_id, int)
		sql = "DELETE FROM global_task_list WHERE robot = %s AND status = 'assigned'"
		val = (robot_id,)
		try:
			self.conn.reconnect()
			self.cursor.execute(sql, val)
			self.conn.commit()
			return True
		except:
			print("Database not alive12")
			return False

	### Update database ###

	def sql_update_robot(self, id, robot): 
		assert isinstance(id, int)
		assert isinstance(robot, dict)
		sql = """UPDATE
					global_robot_list
				SET
					id = %s,
					ip = %s,
					port = %s,
					x_loc = %s,
					y_loc = %s,
					theta = %s,
					speed = %s,
					node = %s,
					status = %s,
					battery_status = %s,
					sql_queries = %s,
					traveled_cost = %s,
					traveled_dist = %s, 
					charged_time = %s,
					congestions = %s,
					next_node = %s,
					current_path = %s,
					total_path = %s,
					task_executing_dist = %s,
					task_executing_cost = %s,
					local_task_list_dist = %s,
					local_task_list_cost = %s,
					time_now = %s
				WHERE
					id = {}
				""".format(id)
		val = tuple(robot.values())
		try:
			self.cursor.execute(sql, val)
			self.conn.commit()
			self.sql_queries += 1
			return True
		except:
			print("Database not alive13")	
			return False

	def sql_update_task(self, id, task):
		assert isinstance(id, int)
		assert isinstance(task, dict)
		sql = """UPDATE 
					global_task_list 
				SET
			"""
		for key in task.keys():
			sql += key + '= %s,'
		sql = sql[:-1]
		sql += """
				WHERE 
					id = {}
				""".format(id)
		val = tuple(task.values())
		try:
			self.cursor.execute(sql, val)
			self.conn.commit()
			return True
		except:
			print("Database not alive14")	
			return False

	def sql_update_reservations(self, pheromones):
		sql = """UPDATE environmental_agents SET pheromone = %s WHERE id = %s"""
		val = pheromones
		try:
			self.cursor.executemany(sql, val)
			self.conn.commit()
			return True
		except:
			print("Database not alive15")	
			return False