# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright (c) 2019, FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions
#    and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of
#    conditions and the following disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# -- END LICENSE BLOCK ------------------------------------------------

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
    match = re.match(r"( )*(\/\/)(.*)", line)
    if match:
        comment = str(match.group(3)).strip()
    return comment


def rchop(thestring, ending):
    """Removes ending from end if thestring if thestring ens in ending"""
    if thestring.endswith(ending):
        return thestring[:-len(ending)]
    return thestring


class CppParser(object):
    """
    Parses cpp ROS nodes
    """

    template_regex = r'(<\s*(?P<type>[^>\s]*)\s*>)?'
    service_template_regex = r'(<(?P<type>[^,>]+)::Request.*>)?'
    type_regex = r'(?P<type>[^,)]+)'
    name_regex = r'(?P<name>[^,)]*)'
    filler_regex = r'[^,)]+'
    default_regex = r'(?P<default>[^,]+)'
    queue_regex = r'\d+'
    callback_regex = r'(?P<callback>([^,()]+)(\([^()]*\))?([^,)])*)'
    remainder_regex = r'(,\s*(?P<remainder>[^)]+))?'

    # I'd like to get rid of this....
    boost_callback_regex = r'(.*boost::bind\((?P<bind>[^\()]*)\)|(?P<callback>[^,)]*))'

    # Based on http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Subscriber_Options
    subscriber_regex = r"\s*".join(['subscribe', template_regex, r'\(', name_regex,
                                    ',', queue_regex, ',', callback_regex, remainder_regex, r'\)'])
    # print("Subscriber regex: " + subscriber_regex)

    publisher_regex = r"\s*".join(['advertise', template_regex, r'\(', name_regex,
                                   ',', queue_regex, remainder_regex, r'\)'])
    service_regex = r"\s*".join(['advertiseService', service_template_regex, r'\(',
                                 name_regex, ',', boost_callback_regex, remainder_regex, r'\)'])

    param_regex = r"\s*".join([r'(get)?[pP]aram(::get)?', template_regex, r'\(', name_regex, ',',
                               filler_regex, '(\s*,\s*' + default_regex + ')?', r'\)'])
    service_client_regex = r"\s*".join(['serviceClient', template_regex, r'\(', name_regex,
                                        remainder_regex])

    service_client_regex_alt = r"\s*".join(['service::call', r'\(', name_regex, ',', type_regex,
                                            r'\)'])
    action_client_regex = r"\s*".join(['actionlib::SimpleActionClient', template_regex, r'\(',
                                       name_regex, remainder_regex])
    action_regex = r"\s*".join(['SimpleActionServer', template_regex, r'\(', name_regex,
                                remainder_regex, r'\)'])
    # regex for parsing node attributes
    # action_regex = 'actionlib::SimpleActionServer<(?P<type>[^>]*)>\("?(\s*(?P<name>[^",]*)"?[^)]*)(?P<bind>)(?P<default>)\)'

    def __init__(self, node_name, files):
        self.node = Node(node_name)
        self.files = files

        self.parser_fcts = [
            (self.param_regex, Parameter, self.node.add_parameter),
            (self.subscriber_regex, Subscriber, self.node.add_subscriber),
            (self.publisher_regex, Publisher, self.node.add_publisher),
            (self.action_client_regex, ActionClient, self.node.add_action_client),
            (self.service_client_regex, ServiceClient, self.node.add_service_client),
            (self.service_client_regex_alt, ServiceClient, self.node.add_service_client),
            (self.service_regex, Service, self.node.add_service),
            (self.action_regex, Action, self.node.add_action)
        ]
        self.lines = None

        # Need to parse all files belonging to node
        for filepath in self.files:
            with open(filepath) as filecontent:
                self.lines = filecontent.readlines()
                self.filecontent = "".join(self.lines)
                self.parse(filepath)

    def parse(self, filepath):
        """
        Extract and add all relevant features from cpp node including comments on them.
        """
        line_end = '.*\n'
        line_ends = list()
        for match in re.finditer(line_end, self.filecontent):
            line_ends.append(match.end())

        for regex, as_type, add in self.parser_fcts:
            for match in re.finditer(regex, self.filecontent, re.DOTALL | re.MULTILINE):
                line_start = next(i for i in range(len(line_ends)) if line_ends[i] > match.start())
                line_end = next(i for i in range(len(line_ends)) if line_ends[i] > match.end())
                code = "".join(self.lines[line_start:line_end+1])
                # print("Line {}-{}".format(line_start + 1, line_end + 1))
                # print(regex)
                # print(match.group(0))
                # print("Line {}-{}:\n{}".format(line_start + 1, line_end + 1, code))
                item, brackets = self.extract_info(code, as_type, regex)
                if item:
                    filename = filepath.split("/")[-1]
                    item.filename = filename
                    item.line_number = line_start + 1
                    item.code = code.strip()
                    comment = self.search_for_comment(line_start)
                    if comment:
                        item.description = comment
                    add(item)

    def check_command_end(self, string, start_search=0):
        """Checks whether the given line is a full c++ command (Whether there is a ';' in the line
        that is not inside a string)"""

        semicolon = string.find(";", start_search)
        if semicolon > 0:
            num_pre_quotes = string.count("\"", 0, semicolon)
            if num_pre_quotes % 2 == 1:
                return self.check_command_end(string, semicolon + 1)
            return True

        return False

    @staticmethod
    def remove_surrounding_quotes(string):
        ret_str = string
        if string.startswith('\"') and string.endswith('\"'):
            ret_str = string.lstrip('\"').rstrip('\"')
        return ret_str

    @staticmethod
    def concatenate_multiline(string):
        return re.sub(r'\"\s*\"', '', string)

    def extract_info(self, line, as_type, regex):
        """
        Check whether a line contains a topic item matching the given regex
        Returns True if line contains a corresponding item and False otherwise.
        """
        match = re.search(regex, line, re.DOTALL | re.MULTILINE)
        if match:
            name = str(match.group('name'))
            name = self.concatenate_multiline(name)
            if 'default' in match.groupdict().keys():
                if match.group('default'):
                    tmp = str(match.group('default')).replace('\'', '')#.replace('\"', '')
                    default_value = self.remove_surrounding_quotes(tmp)
                else:
                    default_value = None
            if 'type' in match.groupdict().keys():
                datatype = str(match.group('type')).strip('"').replace(",", "")
                if datatype == "None":
                    if "callback" in match.groupdict().keys():
                        if match.group("callback"):
                            signature = self.get_func_signature(match.group('callback').lstrip("&"))
                            if signature:
                                datatype = signature.split(" ")[0]
                                if datatype.strip() == "const":
                                    datatype = signature.split(" ")[1]
                                datatype = datatype.strip("&")
                                datatype = rchop(datatype, "Ptr")
                                datatype = rchop(datatype, "Const")
                                datatype = rchop(datatype, "Request")
                                datatype = datatype.strip(":")
                    if "bind" in match.groupdict().keys() and match.group("bind"):
                        func_call = match.group("bind").split(",")[0]
                        signature = self.get_func_signature(func_call.lstrip("&"))
                        if signature:
                            datatype = signature.split(" ")[0]
                            if datatype == "const":
                                datatype = signature.split(" ")[1]
                            datatype = datatype.strip("&")
                            datatype = rchop(datatype, "Ptr")
                            datatype = rchop(datatype, "Const")
                            datatype = rchop(datatype, "Request")
                            datatype = datatype.strip(":")
            brackets = line
            if as_type == Parameter:
                return as_type(name, default_value=default_value, datatype=datatype), brackets
            datatype = datatype.replace("::", "/")
            return as_type(name, datatype=datatype), brackets
        return None, None

    def get_func_signature(self, func_name):
        """
        Searches for a function that is named as func_name and extracts its signature
        """
        # print("Searching signature of function {}".format(func_name))
        sig_regex = func_name + r"\((?P<signature>[^)]+)\)"
        # print(sig_regex)

        lines = list()
        full_line = ""
        for line in self.lines:
            if not self.check_command_end(line):
                lines.append(line.strip(' ').strip('\n'))
                full_line = " ".join(lines)
                continue
            match = re.search(sig_regex, full_line)
            if match:
                # print("Found signature: '{}'".format(match.group("signature").strip()))
                return match.group("signature").strip()
        return None

    def search_for_comment(self, linenumber):
        """
        searches for commented lines right above the given linenumber until one line without comment
        is found
        """
        still_comment = True
        # We need to start one line before the starting line
        line_of_comment = linenumber - 1
        # print("Searching for comment starting in line {}".format(line_of_comment+1))
        comment_lines = list()
        while still_comment:
            comm_line = extract_comment(self.lines[line_of_comment])
            if comm_line is None:
                still_comment = False
            else:
                comment_lines.insert(0, comm_line)
                line_of_comment -= 1
        return " ".join(comment_lines)
