"""Small module to check a python file for ros api items"""

from __future__ import print_function

import re
import os

import catkin_doc.node


class PythonParser(object):
    """Parser for python nodes which fills the node representation"""
    def __init__(self, filename):
        self.node = catkin_doc.node.Node(filename)
        self.parser_fcts = [(self.extract_param, self.add_param),
                            (self.extract_sub, self.add_sub),
                            (self.extract_pub, self.add_pub),
                            (self.extract_action_client, self.add_action_client),
                            (self.extract_service_client, self.add_service_client),
                            (self.extract_service, self.add_service),
                            (self.extract_action, self. add_action)]
        with open(filename) as filecontent:
            self.lines = filecontent.readlines()



    def parse(self):
        """
        Parses the lines extracted from file in init method.
        Therefore extract and add all relevant features from python node including comments on them.
        """
        #TODO: find out if there is a nicer way to handel statements over more lines than concatenating lines
        linenumber = 0
        while linenumber < len(self.lines) - 2:
            line = self.lines[linenumber].lstrip(' ').strip('\n') + ' ' +  self.lines[linenumber+1].lstrip(' ').strip('\n') + ' ' + self.lines[linenumber+2].lstrip(' ')

            for extract,add in self.parser_fcts:
                success, key, value = extract(line)
                if success:
                    comment = self.search_for_comment(linenumber)
                    if comment == '':
                        filename = file.split("/")[-1]
                        comment = 'Please add description. See ' + filename + ' linenumber: ' + str(linenumber+1)
                    add(key, value, comment)

            linenumber += 1


    def search_for_comment(self, linenumber):
        """
        searches for commented lines right above the given linenumber until one line without comment is found
        """
        still_comment = True
        comment = ''
        line_of_comment = linenumber -1
        while still_comment:
            comm_line = self.extract_comment(self.lines[line_of_comment])
            if comm_line:
                comment = comm_line + " "  + comment
                line_of_comment -= 1
            else:
                still_comment = False
        return comment



    def extract_param(self, line):
        """
        Check whether a line contains a parameter definition and extract parameters.
        Returns True when parameter is found, False otherwise. Parameter name and value will be
        saved in members.
        """
        match = re.search("(get_param\()\ ?(\'|\")?(\S+)(\'|\")?(, (\S+))?\)", line)
        if match:
            print(match.groups())
            parameter_name = str(match.group(3)).strip('\'')
            print('Parameter name: ', parameter_name)

            parameter_value = str(match.group(6)).strip('\'')
            print('Default value: ', parameter_value)
            return True, parameter_name, parameter_value
        return False, None, None

    def add_param(self, name, value, comment):
        """
        Add given param + value + comment to node
        """
        self.node.add_parameter(name, value, comment)

    def extract_sub(self, line):
        """
        Check whether a line contains a Subscriber to a topic.
        Returns True if line contains a subscriber and False otherwise.
        """
        match = re.search("(Subscriber\()\ ?(\'|\")?(\S+)(\'|\")?(, (\S+))(, (\S+))\)", line)
        if match:
            print(match.groups())
            topic = str(match.group(3))
            print('Subscribed topic: ', topic)

            topic_type = str(match.group(6))
            print('Msg type on topic: ', topic_type)

            return True, topic, topic_type
        return False, None, None

    def add_sub(self, topic, msg_type, comment):
        """
        Add given subscriber + msg_type + comment to node
        """
        self.node.add_subscriber(topic, msg_type, comment)


    def extract_pub(self, line):
        """
        Check whether a line contains a Publisher to a topic.
        Returns True if line contains a Publisher and False otherwise.
        """
        match = re.search("(Publisher\()\ ?(\'|\")?(\S+)(\'|\")?(, (\S+))(, (\S+))+\)", line)
        if match:
            print(match.groups())
            topic = str(match.group(3))
            print('Published topic: ', topic)

            topic_type = str(match.group(6))
            print('Msg type on topic: ', topic_type)
            return True, topic, topic_type
        return False, None, None

    def add_pub(self, topic, msg_type, comment):
        """
        Add given publisher + msg_type + comment to node
        """
        self.node.add_publisher(topic, msg_type, comment)

    def extract_action_client(self, line):
        """
        Check whether a line contains an action client.
        Returns True if line contains an action client and False otherwise.
        """
        match = re.search("(SimpleActionClient\()\ ?(\'|\")?(\S+)(\'|\")?(, (\S+))\)", line)
        if match:
            print(match.groups())
            topic = str(match.group(3))
            print('Action topic: ', topic)

            action = str(match.group(6))
            print('Action: ', action)
            return True, topic, action
        return False, None, None

    def add_action_client(self, topic, action, comment):
        """
        Add given topic + action + comment to node
        """
        self.node.add_action_client(topic, action, comment)


    def extract_service_client(self, line):
        """
        Check whether a line contains an service client.
        Returns True if line contains an service client and False otherwise.
        """
        match = re.search("(ServiceProxy\()\ ?(\'|\")?(\S+)(\'|\")?(, (\S+))\)", line)
        if match:
            print(match.groups())
            topic = str(match.group(3))
            print('Service topic: ', topic)

            type = str(match.group(6))
            print('used srv-type: ', type)
            return True, topic, type
        return False, None, None

    def add_service_client(self, topic, type, comment):
        """
        Adds service client to node with given topic + type + comment
        """
        self.node.add_service_client(topic, type, comment)


    def extract_service(self, line):
        """
        Check whether a line contains an service.
        Returns True if line contains an service and False otherwise.
        """
        match = re.search("(Service\()\ ?(\'|\")?(\S+)(\'|\")?(, (\S+))(, (\S+))+\)", line)
        if match:
            print(match.groups())
            name = str(match.group(3))
            print('Service: ', name)

            type = str(match.group(6))
            print('used srv-type: ', type)

            return True, name, type
        return False, None, None

    def add_service(self, name, type, comment):
        """
        Adds service to node with given name, type and comment
        """
        self.node.add_service(name, type, comment)

    def extract_action(self, line):
        """
        Check whether a line contains an action.
        Returns True if line contains an action and False otherwise.
        """
        match = re.search("(SimpleActionServer\()\ ?(\'|\")?(\S+)(\'|\")?(, (\S+))(, (\S+))+\)", line)
        if match:
            print(match.groups())
            name = str(match.group(3))
            print('Action: ', name)

            type = str(match.group(6))
            print('used action-type: ', type)

            return True, name, type
        return False, None, None

    def add_action(self, name, type, comment):
        """
        Add action to node with given name, type and comment
        """
        self.node.add_action(name, type, comment)

    def extract_comment(self, line):
        """
        Checks whether the line contains an comment
        If so method returns comment, None otherwise
        """
        comment = None
        match = re.match("( )*(#)(.*)", line)
        if match:
            comment = str(match.group(3))
        return comment

    def get_node(self):
        """
        Returns the Node Object
        """
        return self.node
