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

"""Module to parse an existing mark down documentation"""
from __future__ import print_function
import re

import catkin_doc.datastructures as ds
from catkin_doc.datastructures.doc_object import DocObject
from catkin_doc.datastructures.parameter import Parameter, LaunchArgument
from catkin_doc.datastructures.package import Package
from catkin_doc.datastructures.topic import Publisher, Subscriber
from catkin_doc.datastructures.node import Node
from catkin_doc.datastructures.action import Action, ActionClient
from catkin_doc.datastructures.service import Service, ServiceClient
from catkin_doc.datastructures.launchfile import LaunchFile


class DocSection(object):
    """Small helper class representing a hierarchy level inside a doc file"""
    symbol_regex = r'(?P<symbol>Symbol:)'
    name_regex = r'(?P<name>[^*\r\n]+)'
    default_regex = r'(default:\s*(?P<default>.*))'
    type_only_regex = r'(?!http).+'
    type_with_url_regex = r'\[(?P<type_part>.*)\]\((?P<url_part>.*)\)'
    type_regex = r'(?P<type>(' + type_with_url_regex + ')|(' + type_only_regex + '))'
    required_regex = r'((?P<required>Required))'

    def __init__(self, lines, doc_object_type=DocObject, level=0):
        self.lines = lines
        self.level = level
        self.package_t = doc_object_type
        self.children_t = doc_object_type
        self.children = dict()
        self.name = None

        self.line_iterator = enumerate(self.lines)

        self.parameter_style_types = [Parameter, Publisher, Subscriber, Service,
                                      ServiceClient, Action, ActionClient, LaunchArgument]

        # Only needed for some types
        self.type_info = None
        self.default_value = None
        self.var_name = False


        self.title_regex = "^#{" + str(level + 1) + "} ?(?P<name>[^#].*)"
        if self.package_t in self.parameter_style_types:
            self.title_regex = r'\s*'.join(['^#{' + str(level + 1) + '}',
                                            self.symbol_regex + '?',
                                            self.name_regex,
                                            r'\(',
                                            self.default_regex + '?',
                                            self.type_regex + '?',
                                            self.required_regex + '?',
                                            r'\)',
                                            '$'])
        self.parse_title()
        self.description = ""

        self.sub_regex = "^#{" + str(level + 2) + "} ?(?P<name>[^#].*)"

        if self.children_t in self.parameter_style_types:
            self.sub_regex = r'^#{' + str(level + 2) + r'}\s*' + self.symbol_regex + r'?\s*' + self.name_regex
            # print(self.sub_regex)
        self.parse_children()

    def parse_title(self):
        """
        Parses the headline of the current section. Note that in case of parameter_style_types this
        is just an enumeration item.
        """
        # print("title regex: {}".format(self.title_regex))
        for _, line in self.line_iterator:
            match = re.search(self.title_regex, line)
            if match:
                # print("{}Found current level's title: {}".format(self.level*" ", match.group('name')))
                self.name = match.group('name').strip()
                self.children_t = ds.create_doc_object(self.name)
                try:
                    if match.group('symbol') == r'Symbol:':
                        self.var_name = True
                    self.default_value = match.group('default')
                    if match.group('type'):
                        self.type_info = self.parse_type(match.group('type'))
                    else:
                        self.type_info = match.group('required')

                except IndexError:
                    # If our regex doesn't contain these groups, ignore
                    pass
                # print(self)
                return self.name
        return None

    def parse_children(self):
        """
        Parse the current section's text for children and recursively creates a new DocSection for
        each.
        """
        sub_lines = None
        for _, line in self.line_iterator:
            match = re.search(self.sub_regex, line)
            if match:
                name = match.group('name')
                # print("{}Found child: {}".format(self.level * " ", name))
                sub_lines, _ = self.get_sub_lines()
                if sub_lines:
                    self.children[name] = DocSection(
                        [line] + sub_lines, doc_object_type=self.children_t, level=self.level + 1)
            elif line.strip():
                if self.description:
                    self.description = " ".join([self.description, line.strip()])
                else:
                    self.description = line.strip()

    def parse_type(self, type_str):
        """Parses the type from the substring that matched the 'type' part. This can be either type
        only or [type](url)"""
        match = re.match(self.type_with_url_regex, type_str)
        if match:
            return match.group('type_part')
        return type_str

    def get_sub_lines(self):
        """
        Step forward in lines until the next section is found. With this we know which lines belong
        to the current section.
        """
        sub_lines = list()
        end_line = len(self.lines) - 1
        for line_number, line in self.line_iterator:
            sub_lines.append(line)
            if line_number < len(self.lines) - 1:
                next_line = self.lines[line_number + 1]

                match = re.search(self.sub_regex, next_line)
                if match:
                    end_line = line_number
                    break

        return sub_lines, end_line

    def to_doc_object(self):
        """
        Converts a DocSection into a general DocObject structure
        """
        # print("{}".format(self))
        if ds.get_identifier_for_type(self.package_t) in ds.KEYS:
            # print("DocObject for {}".format(self.name))
            if self.package_t in self.parameter_style_types and self.package_t is not Parameter:
                # print(self.package_t)
                doc_object = self.package_t(name=self.name,
                                            description=self.description,
                                            datatype=self.type_info,
                                            var_name=self.var_name)
            elif self.package_t is Parameter:
                doc_object = self.package_t(name=self.name,
                                            description=self.description,
                                            default_value=self.default_value,
                                            datatype=self.type_info,
                                            var_name=self.var_name)
            else:
                doc_object = self.package_t(name=self.name,
                                            description=self.description,
                                            var_name=self.var_name)

            for child in self.children:
                # print("Processing child '{}'".format(child))
                doc_object.children[child] = self.children[child].to_doc_object()
            return doc_object
        else:
            # print(self.name)
            return [c.to_doc_object() for c in self.children.values()]

    def __str__(self):
        out_str = ""
        prefix = self.level * " "
        if self.name:
            out_str += prefix + "Name: " + self.name + "\n"
        else:
            out_str += prefix + "Name: ===== EMPTY =====\n"
        out_str += prefix + "Description: " + self.description + "\n"
        out_str += prefix + "Type: " + ds.get_identifier_for_type(self.package_t) + "\n"
        if self.type_info:
            out_str += prefix + "msg_type: " + self.type_info + "\n"
        if self.default_value:
            out_str += prefix + "default_value: " + self.default_value + "\n"
        for key in self.children:
            out_str += prefix + key + ":\n" + str(self.children[key])
        return out_str


class MdParser(object):
    """Parser for existing markdown files generated with the catkin doc module
       to fill node representation for update"""

    def __init__(self, filename):
        self.doc = None

        if ".md" in filename:
            with open(filename) as filecontent:
                lines = filecontent.readlines()
                self.doc = DocSection(lines, Package, level=0)
        else:
            raise RuntimeError("This is not a markdown file: {}".format(filename))
