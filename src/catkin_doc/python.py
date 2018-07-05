"""Small module to check a python file for ros api items"""

from __future__ import print_function

import re
import os

import catkin_doc.node


class PythonParser(object):
    """Parser for python nodes which fills the node representation"""
    def __init__(self):
        self._node = catkin_doc.node.Node()



    def init_from_filename(self, filename):
        with open(filename) as filecontent:
            lines = filecontent.readlines()
            for line in lines:
                self.extract_params(line)
        self._node.node_to_md()


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
            self._node.add_parameter(parameter_name)

            parameter_value = str(match.group(4)).strip('\'')
            print('Default value: ', parameter_value)
            if not match.group(4):
               parameter_value = None
            self._node.add_parameter(parameter_name, parameter_value)
            return True
        return False
