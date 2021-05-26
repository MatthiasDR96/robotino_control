class Task:
    """
            A class containing the Task representation
    """

    def __init__(self, args):

        if len(args) == 2:
            self.id = args[0]  # int
            self.node = args[1] # string
            self.priority = 0 # int
            self.robot = -1  # int
            self.message = ''  # string
            self.status = ''  # string
        else:
            self.id = args[0]  # int
            self.node = args[1] # string
            self.priority = args[2] # int
            self.robot = args[3]  # int
            self.message = args[4]  # string
            self.status = args[5]  # string

    def to_log(self):
        return str(self.id) + ";" + str(self.node) + \
               ";" + "R" + str(self.robot) + ";" + str(self.priority) + ";" + str(int(self.picked))

    def to_string(self):
        return '[' + str(self.id) + ", " + str(self.node) + ", " + str(self.priority) + ']'
