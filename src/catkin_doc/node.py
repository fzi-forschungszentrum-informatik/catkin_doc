"""Node representation"""

class Node(object):
    """An abstract representation for a ROS node"""
    def __init__(self):
        self._parameters = dict()
        self._subscriber = dict()
        self._publisher = dict()
        self._file = None

    def add_parameter(self, parameter_name, default_value=None):
        self._parameters[parameter_name] = default_value

    def add_subscriber(self, topic, msg_type):
        self._subscriber[topic] = msg_type

    def add_publisher(self, topic, msg_type):
        self._publisher[topic] = msg_type

    def write_parameters_to_file(self):
        self._file.write("## Parameter\n")
        for param in self._parameters:
          self._file.write(param + ": ")
          if self._parameters[param]:
            self._file.write("default:" + self._parameters[param] + "\n")

    def write_subscriber_to_file(self):
        self._file.write("## Subscriber\n")
        for topic in self._subscriber:
          self._file.write("topic: "+ topic + " msg-type: " + self._subscriber[topic] + "\n")

    def write_publisher_to_file(self):
        self._file.write("## Publisher\n")
        for topic in self._publisher:
          self._file.write("topic: "+ topic + " msg-type: " + self._publisher[topic] + "\n")

    def node_to_md(self):
        self._file = open("README.md", "w+")
        self.write_parameters_to_file()
        self.write_subscriber_to_file()
        self.write_publisher_to_file()
        self._file.close()
