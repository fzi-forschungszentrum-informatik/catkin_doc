"""Module to parse a list of cpp files for ros api items"""

import re
import os
import catkin_doc.node


class CppParser(object):

    def __init__(self, node_name, files):
        self.node = catkin_doc.node.Node(node_name)
        self.files = files
        self.parser_fnct = [(self.extract_param, self.add_param)]
        self.lines = None


    def parse_node(self):
        """parses all files belonging to cpp node"""
        for file in files:
            with open(filename) as filecontent:
                self.lines = filecontent.readlines()

    def extract_param(self, line):
        """
        Check whether a line contains a parameter definition and extract parameters.
        Returns True when parameter is found, False otherwise. Parameter name and value will be
        saved in members.
        """
        match = re.search('param<([^>]*)>\("([^"]*)", [^,]+, ([^\)]+)\)', line)
        if match:
            print(match.groups())
            parameter_name = str(match.group(2)).strip('\'')
            print('Parameter name: ', parameter_name)

            parameter_value = str(match.group(3)).strip('\'')
            print('Default value: ', parameter_value)
            return True, parameter_name, parameter_value

        return False, None, None

    def add_param(self, name, value, comment):
        """
        Add given param + value + comment to node
        """
        self.node.add_parameter(name, value, comment)




