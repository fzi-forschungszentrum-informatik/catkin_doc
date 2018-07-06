"""Small module to check a python file for ros api items"""

from __future__ import print_function

import re
import os

import catkin_doc.node


class PythonParser(object):
    """Parser for python nodes which fills the node representation"""
    def __init__(self):
        self.node = catkin_doc.node.Node()



    def init_from_filename(self, filename):
        with open(filename) as filecontent:
            lines = filecontent.readlines()
            for line in lines:
                self.extract_params(line)
        self.node.node_to_md()


    def extract_params(self, line):
        """
        Check whether a line contains a parameter definition and extract parameters.
        Returns True when parameter is found, False otherwise. Parameter name and value will be
        saved in members.
        """
        match = re.search("(get_param\()(\'|\")(\S+)(\'|\")(, (\S+))?\)", line)
        if match:
            print(match.groups())
            parameter_name = str(match.group(3)).strip('\'')
            print('Parameter name: ', parameter_name)

            parameter_value = str(match.group(6)).strip('\'')
            print('Default value: ', parameter_value)
            if not match.group(4):
               parameter_value = None
            self.node.add_parameter(parameter_name, parameter_value)
            return True
        return False

    def extract_subs(self, line):
        """
        Check whether a line contains a Subscriber to a topic.
        Returns True if line contains a subscriber and False otherwise.
        """
        match = re.search("(Subscriber\()(\'|\")(\S+)(\'|\")(, (\S+))(, (\S+))\)", line)
        if match:
            print(match.groups())
            topic = str(match.group(3))
            print('Subscribed topic: ', topic)

            topic_type = str(match.group(6))
            print('Msg type on topic: ', topic_type)
            self.node.add_subscriber(topic, topic_type)
            return True
        return False

    def extract_pubs(self, line):
        """
        Check whether a line contains a Publisher to a topic.
        Returns True if line contains a Publisher and False otherwise.
        """
        match = re.search("(Publisher\()(\'|\")(\S+)(\'|\")(, (\S+))(, (\S+))+\)", line)
        if match:
            print(match.groups())
            topic = str(match.group(3))
            print('Published topic: ', topic)

            topic_type = str(match.group(6))
            print('Msg type on topic: ', topic_type)
            self.node.add_publisher(topic, topic_type)
            return True
        return False
