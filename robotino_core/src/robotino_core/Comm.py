import socket
import pickle
from mysql.connector import connect, Error

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

	def sql_open(self):
		try:
			cnx = connect(
				host = self.host, 
				user = self.user, 
				password = self.password, 
				database = self.database
			)
			self.__conn = cnx
			self.__cursor = cnx.cursor(dictionary=True, buffered=True)
		except Error as e:
			print("Error %d: %s" % (e.args[0],e.args[1]))

	def sql_close(self):
		self.__cursor.close()
		self.__conn.close() 

	def tcp_server_open(self):
		self.sock_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.sock_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Make address reusable
		self.sock_server.bind((self.ip, self.port))
		self.sock_server.listen()
		self.sock_server.settimeout(None)

	def tcp_server_close(self):
		self.sock_server.close()

	def tcp_server_listen(self):

		# New connection
		conn, _ = self.sock_server.accept()
			
		# Receive the data in small chunks and retransmit it
		data = b""
		while True:
			packet = conn.recv(16)
			if packet:
				data += packet
				conn.sendall(packet)
			else:
				break

		# Convert data to dictionary
		data = pickle.loads(data)

		# Clean up the connection
		conn.close()

		return data

	def tcp_client(self, ip, port, data):

		while True:

			try:

				# Create a TCP/IP socket
				sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

				# Connect the socket to the port where the server is listening
				sock.connect((ip, port))
					
				# Make message
				data = pickle.dumps(data)

				# Send data
				sock.sendall(data)

				# Look for the response
				data = sock.recv(1024)
				data = pickle.loads(data)

				# Look for the response
				#amount_expected = len(data)
				#data = b""
				#while len(data) < amount_expected:
					#packet = sock.recv(16)
					#data += packet
				#data = pickle.loads(data)
				#res = True
				#break

			except(socket.error):
				print("\nConnection failed from " + str(self.port) + " to " + str(port) + " , retrying...")

			finally:
				sock.close()
				return data

	def sql_create_database(self, table):
		assert isinstance(table, str)
		sql = "CREATE DATABASE {}".format(table)
		self.__cursor.execute(sql)

	def sql_show_databases(self):
		sql = "SHOW DATABASES"
		self.__cursor.execute(sql)
		for db in self.__cursor:
			print(db)

	def sql_drop_table(self, table):
		assert isinstance(table, str)
		sql = "DROP TABLE IF EXISTS {}".format(table)
		self.__cursor.execute(sql)

	def sql_create_robot_table(self):
		sql = """
		CREATE TABLE IF NOT EXISTS global_robot_list(
				id INT UNIQUE,
				ip VARCHAR(100),
				port INT,
				x_loc FLOAT,
				y_loc FLOAT,
				theta FLOAT,
				node VARCHAR(100),
				status VARCHAR(100),
				battery_status FLOAT,
				travelled_time FLOAT,
				charged_time FLOAT,
				congestions INT,
				task_executing INT,
				path VARCHAR(100),
				total_path VARCHAR(100)
		)
		"""
		self.__cursor.execute(sql)
		self.__conn.commit()

	def sql_create_task_table(self):
		sql = """
		CREATE TABLE IF NOT EXISTS global_task_list(
				id INT AUTO_INCREMENT PRIMARY KEY,
				node VARCHAR(100),
				priority INT,
				robot INT,
				estimated_start_time FLOAT,
				estimated_end_time FLOAT,
				estimated_duration FLOAT,
				real_start_time VARCHAR(100),
				real_end_time VARCHAR(100),
				real_duration VARCHAR(100),
				progress FOAT,
				message VARCHAR(100),
				status VARCHAR(100)
		)
		"""
		self.__cursor.execute(sql)
		self.__conn.commit()

	def sql_show_table(self, table):
		assert isinstance(table, str)
		sql = "DESCRIBE {}".format(table)
		self.__cursor.execute(sql)
		result = self.__cursor.fetchall()
		for row in result:
			print(row)

	def sql_add_to_table(self, table, item):
		assert isinstance(table, str)
		assert isinstance(item, dict)
		fields = ', '.join(item.keys())
		values = ', '.join(["%s"]*len(item.values()))
		sql = 'INSERT IGNORE INTO {} ({fields}) VALUES ({values})'.format(table, fields=fields, values=values)
		val = tuple(item.values())
		#try:
		self.__cursor.execute(sql, val)
		self.__conn.commit()
		return self.__cursor.lastrowid
		#except:
			#print("Database not alive")
			#return None

	def sql_select_from_table(self, table, criterium, value):
		assert isinstance(table, str)
		assert isinstance(criterium, str)
		sql = "SELECT * FROM {} WHERE {} = %s".format(table, criterium) + " ORDER BY id"
		val = (value,)
		#try:
		self.__conn.reconnect()
		self.__cursor.execute(sql, val)
		result = self.__cursor.fetchall()
		return result
		#except:
			#print("Database not alive")	
			#return None
		
	def sql_select_everything_from_table(self, table):
		assert isinstance(table, str)
		sql = "SELECT * FROM {}".format(table) + " ORDER BY id"
		#try:
		self.__conn.reconnect()
		self.__cursor.execute(sql)
		result = self.__cursor.fetchall()
		return result
		#except:
		#print("Database not alive")	
		#return None

	def sql_delete_from_table(self, table, criterium, value):
		assert isinstance(table, str)
		assert isinstance(criterium, str)
		assert isinstance(value, int)
		sql = "DELETE FROM {} WHERE {} = %s".format(table, criterium)
		val = (value,)
		#try:
		self.__cursor.execute(sql, val)
		self.__conn.commit()
		return True
		#except:
			#print("Database not alive")	
			#return False

	def sql_delete_everything_from_table(self, table):
		assert isinstance(table, str)
		assert isinstance(table, str)
		sql = "DELETE FROM {}".format(table)
		#try:
		self.__cursor.execute(sql)
		self.__conn.commit()
		return True
		#except:
			#print("Database not alive")	
			#return False

	def sql_get_local_task_list(self, robot_id):
		assert isinstance(robot_id, int)
		sql = "SELECT * FROM global_task_list WHERE robot = %s AND status = 'assigned' ORDER BY priority"
		val = (robot_id,)
		#try:
		self.__conn.reconnect()
		self.__cursor.execute(sql, val)
		result = self.__cursor.fetchall()
		return result
		#except:
			#print("Database not alive")	
			#self.sql_open()
			#return None

	def sql_delete_local_task_list(self, robot_id):
		assert isinstance(robot_id, int)
		sql = "DELETE FROM global_task_list WHERE robot = %s AND status = 'assigned'"
		val = (robot_id,)
		#try:
		self.__conn.reconnect()
		self.__cursor.execute(sql, val)
		self.__conn.commit()
		return True
		#except:
			#print("Database not alive")
			#return False

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
					travelled_time = %s,
					charged_time = %s,
					congestions = %s,
					task_executing = %s,
					path = %s,
					total_path = %s,
					time_now = %s
				WHERE
					id = {}
				""".format(id)
		val = tuple(robot.values())
		#try:
		self.__cursor.execute(sql, val)
		self.__conn.commit()
		return True
		#except:
			#print("Database not alive")	
			#return False

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
		#try:
		self.__cursor.execute(sql, val)
		self.__conn.commit()
		return True
		#except:
			#print("Database not alive")	
			#return False

	def sql_update_tasks(self, tasks):
		assert isinstance(tasks, list)
		sql = """UPDATE 
					global_task_list 
				SET 
					robot = %s, 
					status = %s, 
					message = %s, 
					priority = %s,
					estimated_start_time = %s,
					estimated_end_time = %s,
					estimated_duration = %s,
					real_start_time = %s,
					real_end_time = %s,
					real_duration = %s,
					progress = %s
				WHERE 
					id = %s
				"""
		val = tasks
		#try:
		self.__cursor.executemany(sql, val)
		self.__conn.commit()
		return True
		#except:
			#print("Database not alive 1")	
			#return False

	def print_database(self):
		try:
			sql = "SELECT * FROM global_task_list"
			self.__cursor.execute(sql)
			print('Task list')
			for row in self.__cursor:
				print('\t' + str(row))
			sql = "SELECT id, node, status, path FROM global_robot_list"
			self.__cursor.execute(sql)
			print('Robot list')
			for row in self.__cursor:
				print('\t' + str(row))
			print()
		except:
			print("Database not alive")

