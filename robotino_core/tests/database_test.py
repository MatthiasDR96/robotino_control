from mysql.connector import connect, Error
import pandas as pd
import matplotlib.pyplot as plt 
from robotino_core.datatypes.Robot import Robot 
from robotino_core.datatypes.Task import Task

# Connect to database
db = connect(
  host="localhost",
  user="matthias",
  password="matthias",
  database='kb'
)

def connect_database():
  try:
    connection = connect(
      host="localhost",
      user="matthias",
      password="matthias",
      database='kb'
    )
  except Error as e:
    print(e)
  return connection

def create_database(connection, name):
  sql = "CREATE DATABASE {}".format(name)
  with connection.cursor() as cursor:
    cursor.execute(sql)

def show_databases(connection):
  sql = "SHOW DATABASES"
  with connection.cursor() as cursor:
    cursor.execute(sql)
    for db in cursor:
      print(db)

def drop_table(connection, name):
  sql = "DROP TABLE IF EXISTS {}".format(name)
  with connection.cursor() as cursor:
      cursor.execute(sql)

def create_robot_table(connection):
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
  with connection.cursor() as cursor:
    cursor.execute(sql)
    connection.commit()

def create_task_table(connection):
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
  with connection.cursor() as cursor:
    cursor.execute(sql)
    connection.commit()

def show_table(connection, name):
  sql = "DESCRIBE {}".format(name)
  with connection.cursor() as cursor:
    cursor.execute(sql)
    result = cursor.fetchall()
    for row in result:
      print(row)

def add_robot(connection, robot):
  sql = 'INSERT INTO global_robot_list ({fields}) VALUES ({values})'
  fields = ', '.join(list(robot.__dict__.keys())[1:])
  values = ', '.join(['%s' for value in list(robot.__dict__.values())[1:]])
  sql = sql.format(fields=fields, values=values)
  val = tuple(robot.__dict__.values())[1:]
  with connection.cursor() as cursor:
    cursor.execute(sql, val)
    connection.commit()

def add_task(connection, task):
  sql = 'INSERT INTO global_task_list ({fields}) VALUES ({values})'
  fields = ', '.join(list(task.__dict__.keys())[1:])
  values = ', '.join(['%s' for value in list(task.__dict__.values())[1:]])
  sql = sql.format(fields=fields, values=values)
  val = tuple(task.__dict__.values())[1:]
  with connection.cursor() as cursor:
    cursor.execute(sql, val)
    connection.commit()

def delete_from_table(connection, name, criterium, value):
  sql = "DELETE FROM {} WHERE {} = %s".format(name, criterium)
  val = (value,)
  with connection.cursor() as cursor:
    cursor.execute(sql, val)
    connection.commit()

def delete_everything_from_table(connection, name):
  sql = "DELETE FROM {} ".format(name)
  with connection.cursor() as cursor:
    cursor.execute(sql)
    connection.commit()

def select_from_table(connection, name, criterium, value):
  sql = "SELECT * FROM {} WHERE {} = %s".format(name, criterium)
  val = (value,)
  with connection.cursor() as cursor:
    cursor.execute(sql, val)
    result = cursor.fetchall()
  return result

def select_everything_from_table(connection, name):
  sql = "SELECT * FROM {} ".format(name)
  with connection.cursor() as cursor:
    cursor.execute(sql)
    result = cursor.fetchall()
  return result

def update_robot(connection, robot): 
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
  val = (list(robot.__dict__.values())[3:])
  with connection.cursor() as cursor:
    cursor.execute(sql, val)
    connection.commit()

def update_robot1(connection): 
  robot = Robot(1, 'localhost', 10001, 10.0, 30.0, 0.0, 'pos_3')
  strt = ', '.join('{}=%s'.format(k) for k in list(robot.__dict__.keys())[1:])
  sql = "UPDATE global_robot_list SET " + strt + " WHERE id = {}".format(robot.id)
  val = (list(robot.__dict__.values())[1:])
  print(strt)
  print(sql)
  with connection.cursor() as cursor:
    cursor.execute(sql, val)
    connection.commit()


def test():

  # Connect
  db = connect_database()

  # Drop tables
  drop_table(db, 'global_robot_list')
  drop_table(db, 'global_task_list')

  # Create new robot table
  create_robot_table(db)

  # Create new task table
  create_task_table(db)

  # Add robot
  robot = Robot(None, 'localhost', 10001, 10.0, 10.0, 0.0, 'pos_1')
  add_robot(db, robot)
  robot = Robot(None, 'localhost', 10001, 10.0, 30.0, 0.0, 'pos_3')
  add_robot(db, robot)

  # Add task
  task = Task(None, 'pos_5', 0, -1, 0, '', 'unassigned')
  add_task(db, task)
  task = Task(None, 'pos_7', 0, -1, 0, '', 'unassigned')
  add_task(db, task)

  # Select robots
  result = select_everything_from_table(db, 'global_robot_list')
  for row in result:
    print(row)
  print()
  
  # Select tasks
  result = select_everything_from_table(db, 'global_task_list')
  for row in result:
    print(row)
  print()

  # Select task
  result = select_from_table(db, 'global_task_list', 'node', 'pos_5')
  result = select_from_table(db, 'global_task_list', 'status', 'unassigned')
  for row in result:
    print(row)
  print()

  # Update robot
  robot = Robot(1, 'localhost', 10001, 10.0, 30.0, 0.0, 'pos_3')
  update_robot(db, robot)

  # Select robots
  result = select_everything_from_table(db, 'global_robot_list')
  for row in result:
    print(row)
  print()

  # Close connection
  db.close()

def clear_database():

  # Drop tables
  drop_table(db, 'global_robot_list')
  drop_table(db, 'global_task_list')

  # Create new robot table
  create_robot_table(db)

  # Create new task table
  create_task_table(db)
