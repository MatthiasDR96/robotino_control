import socket
import pickle
import mysql.connector

class Comm:
    """
        A library containing the interfaces to communicate with an SQL knowledge base or with other devices via TCP/IP
        When the simulation software want to be used in an experimental setup, it is sufficient to change the methods
        in this library.
    """

    def __init__(self, ip, port):

        # Address
        self.ip = ip
        self.port = port

        # SQL connection
        self._conn = mysql.connector.connect(
            host="localhost",
            user="matthias",
            password="matthias",
            database='kb'
        )

        # SQL cursor
        self._cursor = self._conn.cursor()


    def tcp_server(self):

        # Create a TCP/IP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Bind address to socket
        sock.bind((self.ip, self.port))

        # Actas server
        sock.listen()

        try:

            # New connection
            connection, client_address = sock.accept()
            
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

        except(ConnectionRefusedError):
            print("\nConnection refused from " + str(self.port) + " to " + str(port))

        finally:
            sock.close()
            return res

    def sql_add_to_table(self, name, item):
        self._conn.reconnect()
        fields = ', '.join(list(item.__dict__.keys())[1:])
        values = ', '.join(['%s' for value in list(item.__dict__.values())[1:]])
        sql = 'INSERT INTO {} ({fields}) VALUES ({values})'.format(name, fields=fields, values=values)
        val = tuple(item.__dict__.values())[1:]
        self._cursor.execute(sql, val)
        self._conn.commit()
        return self._cursor.lastrowid

    def sql_select_from_table(self, name, criterium, value):
        self._conn.reconnect()
        sql = "SELECT * FROM {} WHERE {} = %s".format(name, criterium) + " ORDER BY id"
        val = (value,)
        with self._conn.cursor(buffered=True) as cursor:
            cursor.execute(sql, val)
            result = cursor.fetchall()
        return result

    def sql_select_everything_from_table(self, name):
        self._conn.reconnect()
        sql = "SELECT * FROM {}".format(name) + " ORDER BY id"
        with self._conn.cursor(buffered=True) as cursor:
            cursor.execute(sql)
            result = cursor.fetchall()
        return result

    def sql_delete_from_table(self, name, criterium, value):
        self._conn.reconnect()
        sql = "DELETE FROM {} WHERE {} = %s".format(name, criterium)
        val = (value,)
        with self._conn.cursor(buffered=True) as cursor:
            cursor.execute(sql, val)
            self._conn.commit()

    def sql_delete_everything_from_table(self, name):
        self._conn.reconnect()
        sql = "DELETE FROM {}".format(name)
        with connection.cursor(buffered=True) as cursor:
            cursor.execute(sql)
            connection.commit()

    def sql_get_local_task_list(self, robot_id):
        connection.reconnect()
        sql = "SELECT * FROM global_task_list WHERE robot = %s AND status = 'assigned' ORDER BY id"
        val = (robot_id,)
        with connection.cursor(buffered=True) as cursor:
            cursor.execute(sql, val)
            result = cursor.fetchall()
        return result

    def sql_delete_local_task_list(self, robot_id):
        connection.reconnect()
        sql = "DELETE FROM global_task_list WHERE robot = %s AND status = 'assigned'"
        val = (robot_id,)
        with connection.cursor(buffered=True) as cursor:
            cursor.execute(sql, val)
            connection.commit()

    def sql_get_task_to_execute(self, robot_id):
        connection.reconnect()
        sql = "SELECT * FROM global_task_list WHERE robot = %s AND priority = 1 AND status = 'assigned'"
        val = (robot_id,)
        with connection.cursor(buffered=True) as cursor:
            cursor.execute(sql, val)
            result = cursor.fetchall()
        return result

    def sql_update_robot(sef, robot): 
        connection.reconnect()
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
                """.format(robot.id)
        val = tuple(robot.__dict__.values())[3:]
        with connection.cursor(buffered=True) as cursor:
            cursor.execute(sql, val)
            connection.commit()

    def sql_update_task(self, task_id, robot_id, status, message, priority):
        connection.reconnect()
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
        val = (robot_id, status, message, priority, task_id)
        with connection.cursor(buffered=True) as cursor:
            cursor.execute(sql, val)
            connection.commit()

    def sql_update_tasks(self, task_id, robot_id, status, message, priority):
        connection.reconnect()
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
        val = [(robot_id[i], status, message, priority[i], task_id[i]) for i in range(len(task_id))]
        with connection.cursor(buffered=True) as cursor:
            cursor.execute(sql, val)
            connection.commit()

    def print_database(self):
        connection.reconnect()
        sql = "SELECT * FROM global_task_list"
        with connection.cursor(dictionary=True, buffered=True) as cursor:
            cursor.execute(sql)
            print('Task list')
            for row in cursor:
                print('\t' + str(row))
        sql = "SELECT id, node, status, path FROM global_robot_list"
        with connection.cursor(dictionary=True, buffered=True) as cursor:
            cursor.execute(sql)
            print('Robot list')
            for row in cursor:
                print('\t' + str(row))
            print()

