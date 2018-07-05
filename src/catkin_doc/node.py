"""Node representation"""

class Node(object):
    """An abstract representation for a ROS node"""
    def __init__(self):
        self._parameters = list()
        self._default_values = dict()

    def add_parameter(self, parameter_name, default_value=None):
        self._parameters.append(parameter_name)
        if default_value:
            self._parameters[parameter_name] = default_value
