"""Small module to check a python file for ros api items"""

from __future__ import print_function

import re
import os

from catkin_doc.datastructures.node import Node
from catkin_doc.datastructures.parameter import Parameter
from catkin_doc.datastructures.topic import Topic


def extract_param(line):
    """
    Check whether a line contains a parameter definition and extract parameters.
    Returns True when parameter is found, False otherwise. Parameter name and value will be
    saved in members.
    """
    match = re.search("get_param\(\ ?(\S+)(,\ ?(\S+)?)?\)", line)
    if match:
        name = str(match.group(1)).replace('\'', '\"')

        datatype = str(match.group(3)).strip('\'').strip('\"')
        brackets = str(match.group(0))
        return Parameter(name, datatype=datatype), brackets
    return None, None


def extract_topic_info(line, regex):
    """
    Check whether a line contains a topic item matching the given regex
    Returns True if line contains a corresponding item and False otherwise.
    """
    match = re.search(regex, line)
    if match:
        topic = str(match.group(1)).replace('\'', '\"')

        topic_type = str(match.group(3)).strip('\'').strip('\"')
        brackets = str(match.group(0))
        return Topic(topic, datatype=topic_type), brackets
    return None, None


def extract_sub(line):
    """
    Check whether a line contains a Subscriber to a topic.
    Returns True if line contains a subscriber and False otherwise.
    """
    regex = "Subscriber\(\ ?(\S+)(, ?(\S+))(, ?(\S+))\)"
    return extract_topic_info(line, regex)


def extract_pub(line):
    """
    Check whether a line contains a Publisher to a topic.
    Returns True if line contains a Publisher and False otherwise.
    """
    regex = "Publisher\(\ ?(\S+)(, ?(\S+))(, ?(\S+))+\)"
    return extract_topic_info(line, regex)


def extract_action_client(line):
    """
    Check whether a line contains an action client.
    Returns True if line contains an action client and False otherwise.
    """
    regex = "SimpleActionClient\(\ ?(\S+)(, ?(\S+))\)"
    return extract_topic_info(line, regex)


def extract_service_client(line):
    """
    Check whether a line contains an service client.
    Returns True if line contains an service client and False otherwise.
    """
    regex = "ServiceProxy\(\ ?(\S+)(, ?(\S+))\)"
    return extract_topic_info(line, regex)


def extract_service(line):
    """
    Check whether a line contains an service.
    Returns True if line contains an service and False otherwise.
    """
    regex = "Service\(\ ?(\S+)(, ?(\S+))(, ?(\S+))+\)"
    return extract_topic_info(line, regex)


def extract_action(line):
    """
    Check whether a line contains an action.
    Returns True if line contains an action and False otherwise.
    """
    regex = "SimpleActionServer\(\ ?(\S+)(, ?(\S+))(, ?(\S+))+\)"
    return extract_topic_info(line, regex)


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

    def __init__(self, filename):
        node_name = filename.split('/')[-1].strip(".py")
        self.node = Node(node_name)
        self.filename = filename.split('/')[-1]
        self.parser_fcts = [(extract_param, self.node.add_parameter),
                            (extract_sub, self.node.add_subscriber),
                            (extract_pub, self.node.add_publisher),
                            (extract_action_client, self.node.add_action_client),
                            (extract_service_client, self.node.add_service_client),
                            (extract_service, self.node.add_service),
                            (extract_action, self.node.add_action)]
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

            for extract, add in self.parser_fcts:
                item, brackets = extract(line)
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
