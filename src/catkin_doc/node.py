"""Node representation"""

class Node(object):
    """An abstract representation for a ROS node"""
    def __init__(self):
        self._parameters = dict()
        self._file = None

    def add_parameter(self, parameter_name, default_value=None):
        self._parameters[parameter_name] = default_value

    def node_to_md(self):
        self._file = open("README.md", "w+")
        self.write_parameters_to_file()
        self._file.close()

    def write_parameters_to_file(self):
        self._file.write("## Parameter\n")
        for param in self._parameters:
          self._file.write(param + ": ")
          if self._parameters[param]:
            self._file.write("default:" + self._parameters[param] + "\n")
