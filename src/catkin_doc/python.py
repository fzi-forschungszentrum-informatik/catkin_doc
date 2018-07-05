"""Small module to check a python file for ros api items"""

from __future__ import print_function

import re
import os

import catkin_doc.node


class PythonNode(catkin_doc.node.Node):
    """representation of a python node"""
    def __init__(self):
        super(PythonNode, self).__init__()


    def init_from_filename(self, filename):
        with open(filename) as filecontent:
            lines = filecontent.readlines()
            for line in lines:
                self.extract_params(line)


    def extract_params(self, line):
        """
        Check whether a line contains a parameter definition and extract parameters.
        Returns True when parameter is found, False otherwise. Parameter name and value will be
        saved in members.
        """
        match = re.search("(get_param\()\'(\S+)\'(, (\S+))?\)", line)
        if match:
            print(match.groups())
            parameter_name = str(match.group(2)).strip('\'')

            print('Parameter name: ', parameter_name)
            self.parameters.append(parameter_name)

            parameter_value = str(match.group(4)).strip('\'')
            print('Default value: ', parameter_value)
            if match.group(4):
                self.default_values[parameter_name] = parameter_value
            return True
        return False
