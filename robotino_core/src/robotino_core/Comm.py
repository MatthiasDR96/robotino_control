from mysql.connector import connect, Error
import ast
import numpy as np
from operator import itemgetter
from robotino_core.Graph import Graph

class Comm:
	"""
		A library containing the interfaces to communicate with an SQL knowledge base or with other devices via TCP/IP
		When the simulation software want to be used in an experimental setup, it is sufficient to change the methods
		in this library.
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

	def sql_open(self):
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

	def sql_create_database(self, table):
		assert isinstance(table, str)
		sql = "CREATE DATABASE {}".format(table)
		self.cursor.execute(sql)

	def sql_show_databases(self):
		sql = "SHOW DATABASES"
		self.cursor.execute(sql)
		for db in self.cursor:
			print(db)

	def sql_drop_table(self, table):
		assert isinstance(table, str)
		sql = "DROP TABLE IF EXISTS {}".format(table)
		self.cursor.execute(sql)

	def sql_create_robot_table(self, name):
		sql = """
		CREATE TABLE IF NOT EXISTS {}(
				id INT UNIQUE,
				ip VARCHAR(100),
				port INT,
				x_loc FLOAT,
				y_loc FLOAT,
				theta FLOAT,
				node VARCHAR(100),
				status VARCHAR(100),
				battery_status FLOAT,
				sql_queries INT,
				traveled_cost FLOAT,
				traveled_dist FLOAT,
				charged_time FLOAT,
				congestions INT,
				task_executing INT,
				moving_to VARCHAR(100),
				path VARCHAR(100),
				total_path LONGTEXT,
				time_now VARCHAR(100),
				task_executing_estimated_cost FLOAT,
				task_executing_estimated_dist FLOAT,
				local_task_list_estimated_cost FLOAT,
				local_task_list_estimated_dist FLOAT,
				local_task_list_end_time  VARCHAR(100)
		)
		""".format(name)
		self.cursor.execute(sql)
		self.conn.commit()

	def sql_create_task_table(self):
		sql = """
		CREATE TABLE IF NOT EXISTS global_task_list(
				id INT AUTO_INCREMENT PRIMARY KEY,
				node VARCHAR(100),
				priority INT,
				robot INT,
				estimated_start_time VARCHAR(100),
				estimated_end_time VARCHAR(100),
				estimated_duration VARCHAR(100),
				real_start_time VARCHAR(100),
				real_end_time VARCHAR(100),
				real_duration VARCHAR(100),
				progress FLOAT,
				message VARCHAR(100),
				status VARCHAR(100),
				approach VARCHAR(100)
		)
		"""
		self.cursor.execute(sql)
		self.conn.commit()

	def sql_create_graph_table(self):
		sql = """
		CREATE TABLE IF NOT EXISTS graph(
				id INT AUTO_INCREMENT PRIMARY KEY,
				name VARCHAR(100),
				x_loc FLOAT,
				y_loc FLOAT,
				neighbors VARCHAR(100)
		)
		"""
		self.cursor.execute(sql)
		self.conn.commit()

	def sql_create_environmental_agents_table(self):
		sql = """
		CREATE TABLE IF NOT EXISTS environmental_agents(
				id INT AUTO_INCREMENT PRIMARY KEY,
				node VARCHAR(100),
				robot INT,
				start_time VARCHAR(100),
				end_time VARCHAR(100),
				pheromone FLOAT
		)
		"""
		self.cursor.execute(sql)
		self.conn.commit()

	def sql_show_table(self, table):
		assert isinstance(table, str)
		sql = "DESCRIBE {}".format(table)
		self.cursor.execute(sql)
		result = self.cursor.fetchall()
		for row in result:
			print(row)

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

	def sql_select_from_table(self, table, criterium, value):
		assert isinstance(table, str)
		assert isinstance(criterium, str)
		sql = "SELECT * FROM {} WHERE {} = %s".format(table, criterium) + " ORDER BY id"
		val = (value,)
		try:
			self.conn.reconnect()
			self.cursor.execute(sql, val)
			result = self.cursor.fetchall()
			return result
		except:
			print("Database not alive3")	
			return None
		
	def sql_select_everything_from_table(self, table):
		assert isinstance(table, str)
		sql = "SELECT * FROM {}".format(table) + " ORDER BY id"
		try:
			self.conn.reconnect()
			self.cursor.execute(sql)
			result = self.cursor.fetchall()
			return result
		except:
			print("Database not alive " + str(table))	
			return None

	def sql_get_local_task_list(self, robot_id):
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

	def sql_get_executing_task(self, robot_id):
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

	def get_graph(self):
		items = self.sql_select_everything_from_table('graph')
		graph = Graph()
		node_names = list(map(itemgetter('name'), items))
		node_locations = np.hstack((np.c_[list(map(itemgetter('x_loc'), items))], np.c_[list(map(itemgetter('y_loc'), items))]))
		node_neighbors = [ast.literal_eval(x) for x in list(map(itemgetter('neighbors'), items))]
		graph.create_nodes(node_locations, node_names)
		graph.create_edges(node_names, node_neighbors)
		return graph

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
			print("Database not alive6")	
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
			print("Database not alive7")	
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
			print("Database not alive8")
			return False

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
					node = %s,
					status = %s,
					battery_status = %s,
					sql_queries = %s,
					traveled_cost = %s,
					traveled_dist = %s, 
					charged_time = %s,
					congestions = %s,
					task_executing = %s,
					moving_to = %s,
					path = %s,
					total_path = %s,
					time_now = %s,
					task_executing_estimated_cost = %s,
					task_executing_estimated_dist = %s,
					local_task_list_estimated_cost = %s,
					local_task_list_estimated_dist = %s,
					local_task_list_end_time = %s
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
			print("Database not alive9")	
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
			print("Database not alive10")	
			return False

	def sql_update_tasks(self, tasks):
		assert isinstance(tasks, list)
		sql = """UPDATE 
					global_task_list 
				SET 
					priority = %s,
					robot = %s, 					
					estimated_start_time = %s,
					estimated_end_time = %s,
					estimated_duration = %s,
					real_start_time = %s,
					real_end_time = %s,
					real_duration = %s,
					progress = %s,
					message = %s,
					status = %s, 
					approach = %s,
					task_bids = %s
				WHERE 
					id = %s
				"""
		val = tasks
		try:
			self.cursor.executemany(sql, val)
			self.conn.commit()
			return True
		except:
			print("Database not alive11")	
			return False

	def sql_update_reservations(self, pheromones):
		sql = """UPDATE environmental_agents SET pheromone = %s WHERE id = %s"""
		val = pheromones
		try:
			self.cursor.executemany(sql, val)
			self.conn.commit()
			return True
		except:
			print("Database not alive12")	
			return False

	def sql_print_database(self):
		try:
			sql = "SELECT * FROM global_task_list"
			self.cursor.execute(sql)
			print('Task list')
			for row in self.cursor:
				print('\t' + str(row))
			sql = "SELECT id, node, status, path, total_path, time_now FROM global_robot_list"
			self.cursor.execute(sql)
			print('Robot list')
			for row in self.cursor:
				print('\t' + str(row))
			print()
		except:
			print("Database not alive13")

