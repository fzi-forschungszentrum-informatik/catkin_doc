"""Small module to check a cpp file for ros api items"""

from __future__ import print_function

import re

from catkin_doc.datastructures.node import Node
from catkin_doc.datastructures.parameter import Parameter
from catkin_doc.datastructures.service import Service, ServiceClient
from catkin_doc.datastructures.action import Action, ActionClient
from catkin_doc.datastructures.topic import Subscriber, Publisher


def extract_comment(line):
    """
    Checks whether the line contains an comment
    If so method returns comment, None otherwise
    """
    comment = None
    match = re.match("( )*(//)(.*)", line)
    if match:
        comment = str(match.group(3))
    return comment


class CppParser(object):
    """
    Parses cpp ROS nodes
    """

    # regex for parsing node attributes
    param_regex = 'param(<(?P<type>[^>]*)>)?\(("?(?P<name>[^",]*)"?, ?(([^,)]+),\s*)?(?P<default>[^\)]+))(?P<bind>)\)'
    param_regex_alt1 = 'getParam\(("?(?P<name>[^",]+)"?, ?[^)]+)(?P<bind>)(?P<type>)(?P<default>)\)'
    param_regex_alt2 = 'param::get\((("?(?P<name>[^",]+)"?, ?[^)]+))(?P<bind>)(?P<type>)(?P<default>)\)'
    subscriber_regex = 'subscribe(<(?P<type>[^>]*)>)?\(("?(?P<name>[^",]*)"?, [^)]*)(?P<bind>)(?P<default>)\)'
    publisher_regex = 'advertise(<(?P<type>[^>]*)>)?\(("?(?P<name>[^",]*)"?,[^)]*)(?P<bind>)(?P<default>)\)'
    action_client_regex = 'actionlib::SimpleActionClient<(?P<type>[^>]*)>\((\s*"?(?P<name>[^,)("]*)?"?,?\s*([^,^)]*)?,([^,^)]*))(?P<bind>)(?P<default>)\)'
    service_client_regex = 'serviceClient(<(?P<type>[^>]*)>)?\(("?(?P<name>[^",)]*)"?[^)]*)(?P<bind>)(?P<default>)\)'
    service_client_regex_alt = 'service::call\(("?(?P<name>[^",)]*)"?, (?P<type>[^,)]+))(?P<bind>)(?P<default>)\)'
    action_regex = 'actionlib::SimpleActionServer<(?P<type>[^>]*)>\((\s*(?P<name>[^,]*, ?([^,]*))[^)]*)(?P<bind>)(?P<default>)\)'
    service_regex = 'advertiseService(<(?P<type>[^>]*)>)?\((\s?"?(?P<name>[^",]*)"?,\s?(?P<bind>[^\(]*)[^)]*)(?P<default>)\)'

    def __init__(self, node_name, files):
        self.node = Node(node_name)
        self.files = files

        self.parser_fcts = [
            (self.param_regex, Parameter, self.node.add_parameter),
            (self.param_regex_alt1, Parameter, self.node.add_parameter),
            (self.param_regex_alt2, Parameter, self.node.add_parameter),
            (self.subscriber_regex, Subscriber, self.node.add_subscriber),
            (self.publisher_regex, Publisher, self.node.add_publisher),
            (self.action_client_regex, ActionClient, self.node.add_action_client),
            (self.service_client_regex, ServiceClient, self.node.add_service_client),
            (self.service_client_regex_alt, ServiceClient, self.node.add_service_client),
            (self.service_regex, Service, self.node.add_service),
            (self.action_regex, Action, self.node.add_action)
        ]
        self.lines = None
        self.boost_binds = dict()

        # Need to parse all files belonging to node
        for filepath in self.files:
            with open(filepath) as filecontent:
                self.lines = filecontent.readlines()
                self.parse(filepath)

    def parse(self, filepath):
        """
        Extract and add all relevant features from cpp node including comments on them.
        """
        commands_generator = self.get_commands()
        for line, linenumber in commands_generator:
            self.extract_boost_bind(line)
            for regex, as_type, add in self.parser_fcts:
                item, brackets = self.extract_info(line, as_type, regex)
                if item:
                    comment = self.search_for_comment(linenumber)
                    if comment == '':
                        filename = filepath.split("/")[-1]
                        item.filename = filename
                        item.line_number = linenumber
                        item.code = brackets
                    else:
                        item.description = comment
                    add(item)

    def get_commands(self):
        """
        Yields all command lines from a file. Assumes that only one semicolon is in one line.
        Also, this will concatenate things such as if, while, etc.
        """
        linenumber = 0
        first_line = 1
        lines = list()
        while linenumber < len(self.lines):
            if self.lines[linenumber].lstrip(' ').startswith("//"):
                if not lines:
                    first_line = linenumber + 2
            else:
                lines.append(self.lines[linenumber].lstrip(' ').strip('\n'))
                full_line = " ".join(lines)
                if self.check_command_end(full_line):
                    yield full_line, first_line
                    lines = list()
                    first_line = linenumber + 2
            linenumber += 1

    @staticmethod
    def check_command_end(string):
        """Checks whether the given line is a full c++ command (Whether there is a ';' in the line
        that is not inside a string)"""

        semicolon = string.find(";")
        if semicolon > 0:
            num_pre_quotes = string.count("\"", 0, semicolon)
            if num_pre_quotes % 2 == 1:
                # TODO: Check if one or more quotes are escaped
                return False
            return True

        return False

    def extract_info(self, line, as_type, regex):
        """
        Check whether a line contains a topic item matching the given regex
        Returns True if line contains a corresponding item and False otherwise.
        """
        match = re.search(regex, line)
        if match:
            name = str(match.group('name'))
            default_value = str(match.group('default')).replace('\'', '\"')
            datatype = str(match.group('type')).strip('"').replace(",", "")
            brackets = line
            bind = str(match.group('bind'))
            if bind and bind in self.boost_binds:
                datatype = self.boost_binds[bind]
            if as_type == Parameter:
                return as_type(name, default_value=default_value), brackets
            datatype = datatype.replace("::", "/")
            return as_type(name, datatype=datatype), brackets
        return None, None

    def extract_boost_bind(self, line):
        """
        Function to parse boost bindings and saves type of binded functions input.
        """
        match = re.search(
            'boost::function<bool\((\S+)::Request&,\s?\S+::Response&\)>\s?(\S+);', line)
        if match:
            service_type = str(match.group(1))
            bind_fct = str(match.group(2))
            self.boost_binds[bind_fct] = service_type
            return True
        return False

    def search_for_comment(self, linenumber):
        """
        searches for commented lines right above the given linenumber until one line without comment
        is found
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
        return comment.strip()
