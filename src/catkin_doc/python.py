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
                self.extract_subs(line)
                self.extract_pubs(line)
                self.extract_action_clients(line)
                self.extract_service_clients(line)
                self.extract_service(line)
                self.extract_action(line)
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

    def extract_action_clients(self, line):
        """
        Check whether a line contains an action client.
        Returns True if line contains an action client and False otherwise.
        """
        match = re.search("(SimpleActionClient\()(\'|\")(\S+)(\'|\")(, (\S+))\)", line)
        if match:
            print(match.groups())
            topic = str(match.group(3))
            print('Action topic: ', topic)

            action = str(match.group(6))
            print('Action: ', action)
            self.node.add_action_client(topic, action)
            return True
        return False

    def extract_service_clients(self, line):
        """
        Check whether a line contains an service client.
        Returns True if line contains an service client and False otherwise.
        """
        match = re.search("(ServiceProxy\()(\'|\")(\S+)(\'|\")(, (\S+))\)", line)
        if match:
            print(match.groups())
            topic = str(match.group(3))
            print('Service topic: ', topic)

            type = str(match.group(6))
            print('used srv-type: ', type)
            self.node.add_service_client(topic, type)
            return True
        return False

    def extract_service(self, line):
        """
        Check whether a line contains an service.
        Returns True if line contains an service and False otherwise.
        """
        match = re.search("(Service\()(\'|\")(\S+)(\'|\")(, (\S+))(, (\S+))+\)", line)
        if match:
            print(match.groups())
            name = str(match.group(3))
            print('Service: ', name)

            type = str(match.group(6))
            print('used srv-type: ', type)
            self.node.add_service(name, type)
            return True
        return False

    def extract_action(self, line):
        """
        Check whether a line contains an action.
        Returns True if line contains an action and False otherwise.
        """
        match = re.search("(SimpleActionServer\()(\'|\")?(\S+)(\'|\")?(, (\S+))(, (\S+))+\)", line)
        if match:
            print(match.groups())
            name = str(match.group(3))
            print('Action: ', name)

            type = str(match.group(6))
            print('used action-type: ', type)
            self.node.add_action(name, type)
            return True
        return False
