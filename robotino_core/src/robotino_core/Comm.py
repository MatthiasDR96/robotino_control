import time
import socket
import pickle
from mysql.connector import connect, Error

class Comm:
	"""
		A library containing the interfaces to communicate with an SQL knowledge base or with other devices via TCP/IP
		When the simulation software want to be used in an experimental setup, it is sufficient to change the methods
		in this library.
	"""

	__host       = None
	__user       = None
	__password   = None
	__database   = None
	__cursor     = None
	__conn		 = None

	def __init__(self, ip, port, host, user, password, database):

		# Params
		self.ip = ip
		self.port = port
		self.__host = host
		self.__user = user
		self.__password = password
		self.__database = database

	def __open(self):
		try:
			cnx = connect(
				host = self.__host, 
				user = self.__user, 
				password = self.__password, 
				database = self.__database
			)
			self.__conn = cnx
			self.__cursor = cnx.cursor(dictionary=True)
		except Error as e:
			print("Matthias: Self made error while opening connection")
			print("Error %d: %s" % (e.args[0],e.args[1]))

	def __close(self):
		self.__cursor.close()
		self.__conn.close()

	def tcp_server(self):

		# Create a TCP/IP socket
		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

		# Bind address to socket
		sock.bind((self.ip, self.port))

		# Actas server
		sock.listen()

		try:

			# New connection
			connection, _ = sock.accept()
			
			# Receive the data in small chunks and retransmit it
			data = b""
			while True:
				packet = connection.recv(16)
				if packet:
					data += packet
					connection.sendall(packet)
				else:
					break
					
		finally:

			# Clean up the connection
			connection.close()

		# Convert data to dictionary
		data = pickle.loads(data)

		return data

	def tcp_client(self, ip, port, data):

		res = False

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
				amount_expected = len(data)
				data = b""
				while len(data) < amount_expected:
					packet = sock.recv(16)
					data += packet
				data = pickle.loads(data)
				res = True
				break

			except(socket.error):
				print("\nConnection failed from " + str(self.port) + " to " + str(port) + " , retrying...")

			finally:
				sock.close()
				return res

	def sql_create_database(self, table):
		assert isinstance(table, str)
		self.__open()
		sql = "CREATE DATABASE {}".format(table)
		self.__cursor.execute(sql)
		self.__close()

	def sql_show_databases(self):
		self.__open()
		sql = "SHOW DATABASES"
		self.__cursor.execute(sql)
		for db in self.__cursor:
			print(db)
		self.__close()

	def sql_drop_table(self, table):
		assert isinstance(table, str)
		self.__open()
		sql = "DROP TABLE IF EXISTS {}".format(table)
		self.__cursor.execute(sql)
		self.__close()

	def sql_create_robot_table(self):
		self.__open()
		sql = """
		CREATE TABLE IF NOT EXISTS global_robot_list(
				id INT AUTO_INCREMENT PRIMARY KEY,
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
		self.__close()

	def sql_create_task_table(self):
		self.__open()
		sql = """
		CREATE TABLE IF NOT EXISTS global_task_list(
				id INT AUTO_INCREMENT PRIMARY KEY,
				node VARCHAR(100),
				priority INT,
				robot INT,
				message VARCHAR(100),
				status VARCHAR(100)
		)
		"""
		self.__cursor.execute(sql)
		self.__conn.commit()
		self.__close()

	def sql_show_table(self, table):
		assert isinstance(table, str)
		self.__open()
		sql = "DESCRIBE {}".format(table)
		self.__cursor.execute(sql)
		result = self.__cursor.fetchall()
		for row in result:
			print(row)
		self.__close()

	def sql_add_to_table(self, table, item):
		assert isinstance(table, str)
		assert isinstance(item, dict)
		self.__open()
		fields = ', '.join(item.keys())
		values = ', '.join(["%s"]*len(item.values()))
		sql = 'INSERT INTO {} ({fields}) VALUES ({values})'.format(table, fields=fields, values=values)
		val = tuple(item.values())
		while True:
			try:
				self.__cursor.execute(sql, val)
				self.__conn.commit()
				break
			except:
				print("Database not alive")
				time.sleep(1)
		result = self.__cursor.lastrowid
		self.__close()
		return result

	def sql_select_from_table(self, table, criterium, value):
		assert isinstance(table, str)
		assert isinstance(criterium, str)
		assert isinstance(value, str)
		self.__open()
		sql = "SELECT * FROM {} WHERE {} = %s".format(table, criterium) + " ORDER BY id"
		val = (value,)
		self.__cursor.execute(sql, val)
		result = self.__cursor.fetchall()
		self.__close()
		return result

	def sql_select_everything_from_table(self, table):
		assert isinstance(table, str)
		self.__open()
		sql = "SELECT * FROM {}".format(table) + " ORDER BY id"
		self.__cursor.execute(sql)
		result = self.__cursor.fetchall()
		self.__close()
		return result

	def sql_delete_from_table(self, table, criterium, value):
		assert isinstance(table, str)
		assert isinstance(criterium, str)
		assert isinstance(value, str)
		self.__open()
		sql = "DELETE FROM {} WHERE {} = %s".format(table, criterium)
		val = (value,)
		self.__cursor.execute(sql, val)
		self.__conn.commit()
		self.__close()

	def sql_delete_everything_from_table(self, table):
		assert isinstance(table, str)
		assert isinstance(table, str)
		self.__open()
		sql = "DELETE FROM {}".format(table)
		self.__cursor.execute(sql)
		self.__conn.commit()
		self.__close()

	def sql_get_local_task_list(self, robot_id):
		assert isinstance(robot_id, int)
		self.__open()
		sql = "SELECT * FROM global_task_list WHERE robot = %s AND status = 'assigned' ORDER BY id"
		val = (robot_id,)
		self.__cursor.execute(sql, val)
		result = self.__cursor.fetchall()
		self.__close()
		return result

	def sql_delete_local_task_list(self, robot_id):
		assert isinstance(robot_id, int)
		self.__open()
		sql = "DELETE FROM global_task_list WHERE robot = %s AND status = 'assigned'"
		val = (robot_id,)
		self.__cursor.execute(sql, val)
		self.__conn.commit()
		self.__close()

	def sql_get_task_to_execute(self, robot_id):
		assert isinstance(robot_id, int)
		self.__open()
		try:
			sql = "SELECT * FROM global_task_list WHERE robot = %s AND priority = 1 AND status = 'assigned'"
			val = (robot_id,)
			self.__cursor.execute(sql, val)
			result = self.__cursor.fetchall()
		except:
			print("Database not alive")
			result = []		
		self.__close()
		return result

	def sql_update_robot(self, id, robot): 
		assert isinstance(id, int)
		assert isinstance(robot, dict)
		self.__open()
		sql = """UPDATE
					global_robot_list
				SET
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
					total_path = %s
				WHERE
					id = {}
				""".format(id)
		val = tuple(robot.values())
		self.__cursor.execute(sql, val)
		self.__conn.commit()
		self.__close()

	def sql_update_task(self, id, task):
		assert isinstance(id, int)
		assert isinstance(task, dict)
		self.__open()
		sql = """UPDATE 
					global_task_list 
				SET 
					robot = %s, 
					status = %s, 
					message = %s, 
					priority = %s 
				WHERE 
					id = {}
				""".format(id)
		val = tuple(task.values())
		self.__cursor.execute(sql, val)
		self.__conn.commit()
		self.__close()

	def sql_update_tasks(self, task_id, robot_id, status, message, priority):
		self.__open()
		sql = """UPDATE 
					global_task_list 
				SET 
					robot = %s, 
					status = %s, 
					message = %s, 
					priority = %s 
				WHERE 
					id = %s
				""".format(task_id)
		val = [tuple(robot_id[i], status, message, priority[i], task_id[i]) for i in range(len(task_id))]
		print(val)
		self.__cursor.execute(sql, val)
		self.__conn.commit()
		self.__close()

	def print_database(self):
		self.__open()
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
		self.__close()

