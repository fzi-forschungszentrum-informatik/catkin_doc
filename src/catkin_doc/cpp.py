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
                            (self.extract_action_client, self.add_action_client),
                            (self.extract_action_server, self.add_action_server)]
        self.lines = None
        self.boost_binds = dict()
        self.parse_node()


    def parse_node(self):
        """parses all files belonging to cpp node"""
        for file in self.files:
            with open(file) as filecontent:
                self.lines = filecontent.readlines()
                self.parse(file)

    def parse(self, file):
        """
        Therefore extract and add all relevant features from python node including comments on them.
        """
        #TODO: find out if there is a nicer way to handel statements over more lines than concatenating lines
        linenumber = 0
        while linenumber < len(self.lines) - 2:
            if not (self.lines[linenumber].lstrip(' ').startswith("//")):
              line = self.lines[linenumber].lstrip(' ').strip('\n') + ' ' +  self.lines[linenumber+1].lstrip(' ').strip('\n') + ' ' + self.lines[linenumber+2].lstrip(' ')
              self.extract_boost_bind(line)
              for extract,add in self.parser_fcts:
                  success, key, value, brackets = extract(line)
                  if success:
                      comment = self.search_for_comment(linenumber)
                      if comment == '':
                          filename = file.split("/")[-1]
                          comment = 'Please add description. See ' + filename + ' line number: ' + str(linenumber+1) + '  \n      ' + "Input in constructor: " + brackets + "  "
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
        Returns True when parameter is found, False otherwise.
        """
        match = re.search('param(<([^>]*)>)?\(("([^"]*)"(, [^,)]+)?, ([^\)]+))\)', line)
        if match:
            parameter_name = str(match.group(4)).strip('\'')
            parameter_value = str(match.group(6)).strip('\'')
            parameter_brackets = str(match.group(3))
            return True, parameter_name, parameter_value, parameter_brackets
        match = re.search('getParam\(("([^"]*)", [^,]+)\)', line)
        if match:
            parameter_name = str(match.group(2)).strip('\'')
            parameter_brackets = str(match.group(1))
            parameter_value = None
            return True, parameter_name, parameter_value, parameter_brackets
        match = re.search('param::get\(("([^"]*)", [^,]+)\)', line)
        if match:
            parameter_name = str(match.group(2)).strip('\'')
            parameter_brackets = str(match.group(1))
            parameter_value = None
            return True, parameter_name, parameter_value, parameter_brackets

        return False, None, None, None

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
        match = re.search('subscribe(<([^>]*)>)?\(("([^"]*)", [^)]*)\)', line)
        if match:
            subscribed_topic = str(match.group(4))
            msg_type = str(match.group(2))
            brackets = str(match.group(3))
            return True, subscribed_topic, msg_type, brackets
        return False, None, None, None

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
        match = re.search('advertise(<([^>]*)>)?\(("([^"]*)",[^)]*)\)', line)
        if match:
            published_topic = str(match.group(4))
            msg_type = str(match.group(2))
            brackets = str(match.group(3))
            return True, published_topic, msg_type, brackets
        return False, None, None, None

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
        match = re.search('advertiseService(<([^>]*)>)?\((\s?"([^"]*)",\s?([^\(]*)[^)]*)\)', line)
        if match:
            service_name = str(match.group(4))
            service_type = str(match.group(2))
            brackets = str(match.group(3))
            bind = str(match.group(5))
            if bind in self.boost_binds:
                service_type = self.boost_binds[bind]
            return True, service_name, service_type, brackets
        return False, None, None, None

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
        match = re.search('serviceClient(<([^>]*)>)?\(("([^"]*)"[^)]*)\)', line)
        if match:
            service_topic = str(match.group(4))
            service_type = str(match.group(2))
            brackets = str(match.group(3))
            return True, service_topic, service_type, brackets

        match = re.search('service::call\(("([^"]*)", ([^,]+))\)', line)
        if match:
            service_topic = str(match.group(2))
            service_type = str(match.group(3))
            brackets = str(match.group(1))
            return True, service_topic, service_type, brackets
        return False, None, None, None


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
        match = re.search('actionlib::SimpleActionClient<([^>]*)>\((\s*([^,^)^(]*)?,?\s*([^,^)]*)?,([^,^)]*))\)', line)
        if match:
            action_type = str(match.group(1)).strip(' ')
            topic = str(match.group(3)).strip('"') + " " + str(match.group(4).strip('"'))
            brackets = str(match.group(2))
            return True, topic, action_type, brackets
        return False, None, None, None

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
        match = re.search('actionlib::SimpleActionServer<([^>]*)>\((\s*([^,]*),\s*([^,]*)[^)]*)\)', line)
        if match:
            action_type = str(match.group(1))
            topic = str(match.group(3)) + " " + str(match.group(4))
            brackets = str(match.group(2))
            return True, topic, action_type, brackets
        return False, None, None, None

    def add_action_server(self, name, type, comment):
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
        match = re.match("( )*(//)(.*)", line)
        if match:
            comment = str(match.group(3))
        return comment

    def extract_boost_bind(self, line):
        """
        Function to parse boost bindings and saves type of binded functions input.
        """
        match = re.search('boost::function<bool\((\S+)::Request&,\s?\S+::Response&\)>\s?(\S+);', line)
        if match:
            service_type = str(match.group(1))
            bind_fct = str(match.group(2))
            self.boost_binds[bind_fct] = service_type
            return True
        return False



