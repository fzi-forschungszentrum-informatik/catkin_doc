"""Module to parse an existing mark down documentation"""

import os
import re

import catkin_doc.datastructures as ds
from catkin_doc.datastructures.doc_object import DocObject
from catkin_doc.datastructures.parameter import Parameter
from catkin_doc.datastructures.package import Package
from catkin_doc.datastructures.topic import Publisher, Subscriber
from catkin_doc.datastructures.node import Node
from catkin_doc.datastructures.action import Action, ActionClient
from catkin_doc.datastructures.service import Service, ServiceClient

class DocSection(object):
    """Small helper class representing a hierarchy level inside a doc file"""
    def __init__(self, lines, doc_object_type=DocObject, level=0):
        self.lines = lines
        self.level = level
        self.package_t = doc_object_type
        self.children_t = doc_object_type
        self.children = dict()

        self.line_iterator = enumerate(self.lines)

        self.parameter_style_types = [Parameter, Publisher, Subscriber, Service,
                ServiceClient, Action, ActionClient]

        # Only needed for some types
        self.type_info = None
        self.default_value = None

        self.title_regex = "^#{" + str(level+1) + "} ?([^#].*)"
        if self.package_t in self.parameter_style_types:
            self.title_regex = "^\s*\*\s*\*\*(.*)\*\*\s*(\(default:\s*(.*)\))?(\(\[([^\]]*)\]\(.*\)\))?(\((.*)\))?$"
        self.parse_title()
        self.description = ""


        self.sub_regex = "^#{" + str(level+2) + "} ?([^#].*)"

        if self.children_t is Node:
            pass
        elif self.children_t in self.parameter_style_types:
            self.sub_regex = "^\s*\*\s*\*\*(.*)\*\*"
        self.parse_children()

    def parse_title(self):
        sub_lines = None
        for line_number, line in self.line_iterator:
            match = re.search(self.title_regex, line)
            if match:
                # print("{}Found current level's title: {}".format(self.level*" ", match.group(1)))
                self.name = match.group(1)
                self.children_t = ds.create_doc_object(match.group(1))
                try:
                    self.default_value = match.group(3)
                    # print(match.group(3))
                    if match.group(5):
                        self.type_info = match.group(5)
                        # print(match.group(5))
                    else:
                        self.type_info = match.group(7)
                        # print(match.group(7))

                except IndexError:
                    # If our regex doesn't contain these groups, ignore
                    pass
                return match.group(1)
            else:
                # If we haven't found a name, continue until we do
                continue

    def parse_children(self):
        sub_lines = None
        for line_number, line in self.line_iterator:
            match = re.search(self.sub_regex, line)
            if match:
                name = match.group(1)
                # print("{}Found child: {}".format(self.level*" ", name))
                sub_lines, end_line = self.get_sub_lines()
                if sub_lines:
                    self.children[name] = DocSection(
                        [line] + sub_lines, doc_object_type=self.children_t, level=self.level+1)
            elif line.strip():
                if self.description:
                    self.description = "\n".join([self.description, line.strip()])
                else:
                    self.description = line.strip()

    def get_sub_lines(self):
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
        if ds.get_identifier_for_type(self.package_t) in ds.KEYS:
            # print("DocObject for {}".format(self.name))
            if self.package_t in self.parameter_style_types and not self.package_t is Parameter:
                doc_object = self.package_t(name=self.name, description=self.description, datatype = self.type_info)
            elif self.package_t is Parameter:
                doc_object = self.package_t(name=self.name, description=self.description, default_value = self.default_value, datatype = self.type_info)
            else:
                doc_object = self.package_t(name=self.name, description=self.description)


            for child in self.children:
                doc_object.children[child] = self.children[child].to_doc_object()
            return doc_object
        else:
            # print(self.name)
            return [c.to_doc_object() for c in self.children.values()]

    def __str__(self):
        out_str = ""
        prefix = self.level * " "
        out_str += prefix + "Name: " + self.name + "\n"
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
            print("This is not a markdown file.")
