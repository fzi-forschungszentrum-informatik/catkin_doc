"""Small module to check a python file for ros api items"""

from __future__ import print_function

import re
import os

import catkin_doc.datastructures as ds


def extract_info(line, as_type, regex):
    """
    Check whether a line contains a topic item matching the given regex
    Returns True if line contains a corresponding item and False otherwise.
    """
    match = re.search(regex, line)
    if match:
        name = str(match.group(1)).replace('\'', '\"')

        datatype = str(match.group(3)).strip('\'').strip('\"')
        brackets = str(match.group(0))
        return as_type(name, datatype=datatype), brackets
    return None, None


def extract_comment(line):
    """
    Checks whether the line contains an comment
    If so method returns comment, None otherwise
    """
    comment = None
    match = re.match("( )*(#)(.*)", line)
    if match:
        comment = str(match.group(3))
    return comment


class PythonParser(object):
    """Parser for python nodes which fills the node representation"""

    param_regex = "get_param\(\ ?(\S+)(,\ ?(\S+)?)?\)"
    subscriber_regex = "Subscriber\(\ ?(\S+)(, ?(\S+))(, ?(\S+))\)"
    publisher_regex = "Publisher\(\ ?(\S+)(, ?(\S+))(, ?(\S+))+\)"
    action_client_regex = "SimpleActionClient\(\ ?(\S+)(, ?(\S+))\)"
    service_client_regex = "ServiceProxy\(\ ?(\S+)(, ?(\S+))\)"
    service_regex = "Service\(\ ?(\S+)(, ?(\S+))(, ?(\S+))+\)"
    action_regex = "SimpleActionServer\(\ ?(\S+)(, ?(\S+))(, ?(\S+))+\)"

    def __init__(self, filename):
        node_name = filename.split('/')[-1].strip(".py")
        self.node = ds.Node(node_name)
        self.filename = filename.split('/')[-1]
        #                    regex        as_type    add_function
        self.parser_fcts = [(self.param_regex, ds.Parameter, self.node.add_parameter),
                            (self.subscriber_regex, ds.Subscriber, self.node.add_subscriber),
                            (self.publisher_regex, ds.Publisher, self.node.add_publisher),
                            (self.action_client_regex, ds.ActionClient, self.node.add_action_client),
                            (self.service_client_regex, ds.ServiceClient, self.node.add_service_client),
                            (self.service_regex, ds.Service, self.node.add_service),
                            (self.action_regex, ds.Action, self.node.add_action)]
        with open(filename) as filecontent:
            self.lines = filecontent.readlines()
        self.parse()

    def parse(self):
        """
        Parses the lines extracted from file in init method.
        Therefore extract and add all relevant features from python node including comments on them.
        """
        # TODO: find out if there is a nicer way to handle statements over more lines than concatenating lines
        linenumber = 0
        while linenumber < len(self.lines) - 2:
            line = self.lines[linenumber].lstrip(' ').strip('\n') + ' ' +\
                self.lines[linenumber + 1].lstrip(' ').strip('\n') + ' ' +\
                self.lines[linenumber + 2].lstrip(' ')

            for regex, as_type, add in self.parser_fcts:
                item, brackets = extract_info(line, as_type, regex)
                if item:
                    comment = self.search_for_comment(linenumber)
                    if comment == '':
                        filename = self.filename.split("/")[-1]
                        item.filename = filename
                        item.line_number = linenumber
                        item.code = brackets
                    else:
                        item.description = comment
                    add(item)

            linenumber += 1

    def search_for_comment(self, linenumber):
        """
        searches for commented lines right above the given linenumber until one line without comment is found
        """
        still_comment = True
        comment = ''
        line_of_comment = linenumber - 1
        while still_comment:
            comm_line = extract_comment(self.lines[line_of_comment])
            if comm_line:
                comment = comm_line + " " + comment
                line_of_comment -= 1
            else:
                still_comment = False
        return comment

    def get_node(self):
        """
        Returns the Node Object
        """
        return self.node
