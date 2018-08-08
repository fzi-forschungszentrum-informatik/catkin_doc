"""Module to parse a list of cpp files for ros api items"""

import re
import os
import catkin_doc.node

class CppParser(object):

    def __init__(self, node_name, files):
        self.node = catkin_doc.node.Node(node_name)
        self.files = files
        self.lines = None
        self.parser_fcts = None

    def parse_node(self):
        """
        Parses every file belonging to the node.
        """
        for file in self.files:
            with open(filename) as filecontent:
                self.lines = filecontent.readlines()



