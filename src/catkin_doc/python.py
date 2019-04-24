"""Small module to check a python file for ros api items"""

from __future__ import print_function

import re
import os

import catkin_doc.node


class PythonParser(object):
    """Parser for python nodes which fills the node representation"""
    def __init__(self, filename):
        node_name = filename.split('/')[-1].strip(".py")
        self.node = catkin_doc.node.Node(node_name)
        self.filename = filename.split('/')[-1]

        # regex for parsing python files
        self.re_param = "get_param\(\ ?(\S+)(,\ ?(\S+)?)?\)"
        self.re_subscriber = "Subscriber\(\ ?(\S+)(, ?(\S+))(, ?(\S+))\)"
        self.re_publisher = "Publisher\(\ ?(\S+)(, ?(\S+))(, ?(\S+))+\)"
        self.re_action_client = "SimpleActionClient\(\ ?(\S+)(, ?(\S+))\)"
        self.re_service_client = "ServiceProxy\(\ ?(\S+)(, ?(\S+))\)"
        self.re_action = "SimpleActionServer\(\ ?(\S+)(, ?(\S+))(, ?(\S+))+\)"
        self.re_service = "Service\(\ ?(\S+)(, ?(\S+))(, ?(\S+))+\)"

        self.parser_fcts = [(self.extract_param, self.add_param),
                            (self.extract_sub, self.add_sub),
                            (self.extract_pub, self.add_pub),
                            (self.extract_action_client, self.add_action_client),
                            (self.extract_service_client, self.add_service_client),
                            (self.extract_service, self.add_service),
                            (self.extract_action, self. add_action)]
        with open(filename) as filecontent:
            self.lines = filecontent.readlines()
        self.parse()



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
                success, key, value, brackets = extract(line)
                if success:
                    comment = self.search_for_comment(linenumber)
                    if comment == '':
                        filename = self.filename.split("/")[-1]
                        comment = 'Please add description. See ' + filename + ' line number: ' + str(linenumber+1) + "\n    Constructor input : " + brackets
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
        match = re.search(self.re_param, line)
        if match:
            parameter_name = str(match.group(1)).replace('\'', '\"')
            parameter_value = str(match.group(3)).replace('\'', '\"')
            brackets = str(match.group(0))
            return True, parameter_name, parameter_value, brackets
        return False, None, None, None

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
        match = re.search(self.re_subscriber, line)
        if match:
            topic = str(match.group(1)).replace('\'', '\"')

            topic_type = str(match.group(3)).strip('\'').strip('\"')
            brackets = str(match.group(0))
            return True, topic, topic_type, brackets
        return False, None, None, None

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
        match = re.search(self.re_publisher, line)
        if match:
            topic = str(match.group(1)).replace('\'', '\"')
            topic_type = str(match.group(3))
            brackets = str(match.group(0))
            return True, topic, topic_type, brackets
        return False, None, None, None

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
        match = re.search(self.re_action_client, line)
        if match:
            topic = str(match.group(1)).replace('\'', '\"')
            action = str(match.group(3))
            brackets = str(match.group(0))
            return True, topic, action, brackets
        return False, None, None, None

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
        match = re.search(self.re_service_client, line)
        if match:
            topic = str(match.group(1)).replace('\'', '\"')
            type = str(match.group(3))
            brackets = str(match.group(0))
            return True, topic, type, brackets
        return False, None, None, None

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
        match = re.search(self.re_service, line)
        if match:
            name = str(match.group(1)).replace('\'', '\"')
            type = str(match.group(3))
            brackets = str(match.group(0))

            return True, name, type, brackets
        return False, None, None, None

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
        match = re.search(self.re_action, line)
        if match:
            name = str(match.group(1)).replace('\'', '\"')
            type = str(match.group(3))
            brackets = str(match.group(0))

            return True, name, type, brackets
        return False, None, None, None

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
