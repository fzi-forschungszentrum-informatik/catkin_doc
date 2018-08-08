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
        #TODO same as in PythonParser concatenating lines is not so nice aslo it is not necessary to parse the header for anything but comments
        for file in self.files:
            with open(filename) as filecontent:
                self.lines = filecontent.readlines()
                linenumber = 0
            while linenumber < len(self.lines) - 2:
                line = self.lines[linenumber].lstrip(' ').strip('\n') + ' ' +  self.lines[linenumber+1].lstrip(' ').strip('\n') + ' ' + self.lines[linenumber+2].lstrip(' ')
                for extract,add in self.parser_fcts:
                    success, key, value = extract(line)
                    if success:
                        comment = None
                        add(key, value, comment)
                linenumber += 1



