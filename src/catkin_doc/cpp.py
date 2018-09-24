"""Module to parse a list of cpp files for ros api items"""

import re
import os
import catkin_doc.node


class CppParser(object):

    def __init__(self, node_name, files):
        self.node = catkin_doc.node.Node(node_name)
        self.files = files
        self.parser_fcts = [(self.extract_param, self.add_param),
                            (self.extract_sub, self.add_sub),
                            (self.extract_pub, self.add_pub),
                            (self.extract_service, self.add_service),
                            (self.extract_service_client, self.add_service_client),
                            (self.extract_action_client, self.add_action_client)]
        self.lines = None


    def parse_node(self):
        """parses all files belonging to cpp node"""
        for file in files:
            with open(filename) as filecontent:
                self.lines = filecontent.readlines()

    def extract_param(self, line):
        """
        Check whether a line contains a parameter definition and extract parameters.
        Returns True when parameter is found, False otherwise.
        """
        match = re.search('param<([^>]*)>\("([^"]*)", [^,]+, ([^\)]+)\)', line)
        if match:
            print(match.groups())
            parameter_name = str(match.group(2)).strip('\'')
            print('Parameter name: ', parameter_name)

            parameter_value = str(match.group(3)).strip('\'')
            print('Default value: ', parameter_value)
            return True, parameter_name, parameter_value
        match = re.search('getParam\("([^"]*)", [^,]+\)', line)
        if match:
            print(match.groups())
            parameter_name = str(match.group(1)).strip('\'')
            print('Parameter name: ', parameter_name)

            parameter_value = None
            return True, parameter_name, parameter_value
        match = re.search('param::get\("([^"]*)", [^,]+\)', line)
        if match:
            print(match.groups())
            parameter_name = str(match.group(1)).strip('\'')
            print('Parameter name: ', parameter_name)

            parameter_value = None
            return True, parameter_name, parameter_value

        return False, None, None

    def add_param(self, name, value, comment):
        """
        Add given param + value + comment to node
        """
        self.node.add_parameter(name, value, comment)

    def extract_sub(self, line):
        """
        Check wheter given line contains a subscriber
        Returns (True, topic, msg_type) if subscriber is found, (False, None, None) otherwise.
        """
        match = re.search('subscribe(<([^>]*)>)?\("([^"]*)",', line)
        if match:
            subscribed_topic = str(match.group(3))
            msg_type = str(match.group(2))
            return True, subscribed_topic, msg_type
        return False, None, None

    def add_sub(self, topic, msg_type, comment):
        """
        Add given subscriber + msg_type + comment to node
        """
        self.node.add_subscriber(topic, msg_type, comment)

    def extract_pub(self, line):
        """
        Check wheter given line contains a publisher
        Returns (True, topic, msg_type) if publisher is found, (False, None, None) otherwise.
        """
        match = re.search('advertise(<([^>]*)>)?\("([^"]*)",', line)
        if match:
            published_topic = str(match.group(3))
            msg_type = str(match.group(2))
            return True, published_topic, msg_type
        return False, None, None

    def add_pub(self, topic, msg_type, comment):
        """
        Add given publisher + msg_type + comment to node
        """
        self.node.add_publisher(topic, msg_type, comment)

    def extract_service(self, line):
        """
        Check wheter a given line contains a Service advertisement
        Returns (True, name, type) if service is found (False, None, None) otherwise.
        """
        match = re.search('advertiseService(<([^>]*)>)?\("([^"]*)",', line)
        if match:
            service_name = str(match.group(3))
            service_type = str(match.group(2))
            return True, service_name, service_type
        return False, None, None

    def add_service(self, name, type, comment):
        """
        Add given name+ type + comment as service to node
        """
        self.node.add_service(name, type, comment)

    def extract_service_client(self, line):
        """
        Check whether a given line contains a service client.
        Returns (True, service_topic, type) if service client is found, (False, None, None) otherwise.
        """
        match = re.search('serviceClient(<([^>]*)>)?\("([^"]*)"', line)
        if match:
            service_topic = str(match.group(3))
            service_type = str(match.group(2))
            return True, service_topic, service_type

        match = re.search('service::call\("([^"]*)", ([^,]+)\)', line)
        if match:
            service_topic = str(match.group(1))
            service_type = str(match.group(2))
            return True, service_topic, service_type
        return False, None, None


    def add_service_client(self, topic, type, comment):
        """
        Adds service client to node with given topic + type + comment
        """
        self.node.add_service_client(topic, type, comment)

    def extract_action_client(self, line):
        """
        Function to extract action clients from given line.
        Returns(True, None,  action_type) if client is found (False, None, None) otherwise
        """
        match = re.search('actionlib::SimpleActionClient<([^>]*)>', line)
        if match:
            action_type = str(match.group(1))
            return True, None, action_type
        return False, None, None

    def add_action_client(self, topic, action, comment):
        """
        Add given topic + action + comment to node
        """
        self.node.add_action_client(topic, action, comment)

    def extract_action_server(self, line):
        """
        Function to extract action server from given line.
        Returns (True, None, action_type) if server is found, (False, None, None) otherwise.
        """
        match = re.search('actionlib::SimpleActionServer<([^>]*)>', line)
        if match:
            action_type = str(match.group(1))
            return True, None, action_type
        return False, None, None

    def add_action_server(self, name, type, comment):
        """
        Add action to node with given name, type and comment
        """
        self.node.add_action(name, type, comment)



