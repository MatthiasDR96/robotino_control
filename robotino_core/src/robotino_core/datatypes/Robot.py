class Robot:
    """
            A class containing the Robot representation
    """

    def __init__(self, args):
        
        if len(args) == 7:
            self.id = args[0]  # int
            self.ip = args[1]  # string
            self.port = args[2]  # int
            self.x_loc = args[3]  # double
            self.y_loc = args[4]  # double
            self.theta = args[5] # double
            self.node = args[6] # string
            self.status = 'IDLE' # string
            self.battery_status = 100.0 # double
            self.travelled_time = 0.0 # double
            self.charged_time = 0.0 # double
            self.congestions = 0  # int
            self.task_executing = -1 # int
            self.path = '[]' # blob
            self.total_path = '[]'  # blob
        else:
            self.id = args[0]  # int
            self.ip = args[1]  # string
            self.port = args[2]  # int
            self.x_loc = args[3]  # double
            self.y_loc = args[4]  # double
            self.theta = args[5] # double
            self.node = args[6] # string
            self.status = args[7] # string
            self.battery_status = args[8] # double
            self.travelled_time = args[9] # double
            self.charged_time = args[10] # double
            self.congestions = args[11]  # int
            self.task_executing = args[12] # int
            self.path = args[13] # blob
            self.total_path = args[14]  # blob

    def to_log(self):
        if self.path:
            path_string = "{"
            for node in self.path:
                path_string += node + ','
            path_string = path_string[:-1]
            path_string += "}"
        else:
            path_string = "{}"
        return str(self.id) + ";" + str(self.robot_location[0]) + ";" + str(self.robot_location[1]) + ";" + str(
            self.status) + ";" + str(self.battery_status) \
               + ";" + str(self.heading_direction) + ";" + path_string + ";" + str(self.travelled_time) \
               + ";" + str(self.charged_time) + ";" + str(self.congestions)

    def to_string(self):
        return '[' + str(self.id) + ", " + str(self.port) + ", " + str(self.node) + ", " + str(self.status) + ", " + \
               str(self.battery_status) + ']'
