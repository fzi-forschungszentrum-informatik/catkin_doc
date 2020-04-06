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

'''
This class serves as a base class for all documentation objects
'''
from __future__ import print_function

import re
import copy

import catkin_doc.datastructures as ds

if hasattr(__builtins__, 'raw_input'):
    input = raw_input


class DocObject(object):
    """Base class for a doc object"""

    def __init__(self, name, description="", var_name=None):
        self.default_description = "Please add description. See {} line number: {}\n\n\t{}"
        self.default_desc_regex = "\s+".join(
            self.default_description.format("(.*)", "(\d+)", "(.*)").split())

        self.name = None
        self.var_name = None
        self.namespace = None
        self.set_name(name, var_name)
        # This is needed for the default description
        self.filename = ""
        self.line_number = None
        self.code = ""
        self.description = self.set_description(description)
        self.children = dict()

    def __eq__(self, other):
        return self.name == other.name and isinstance(other, type(self))

    def __lt__(self, other):
        return self.name.lower() < other.name.lower()

    def __le__(self, other):
        return self.name.lower() <= other.name.lower()

    def __gt__(self, other):
        return self.name.lower() > other.name.lower()

    def __ge__(self, other):
        return self.name.lower() >= other.name.lower()

    @staticmethod
    def indent(text, amount, ch=' '):
        """Add indentation to each line of the input text"""
        padding = amount * ch
        return ''.join(padding+line for line in text.splitlines(True))

    def __str__(self):
        """Default string conversion of DocObject"""
        out_str = self.name + '\n'
        for child_key in self.children:
            out_str += self.indent(child_key, 2) + '\n'
            for child in self.children[child_key]:
                out_str += self.indent(str(child), 4) + '\n'
        return out_str

    def to_string(self, level, formatter):
        """
        Formats the object as markdown text

        :param int level: Level of heading hierarchy
        :param formatter: Formatter to use
        :return: A formatted string for this object formatted by the given formatter
        :rtype: str
        """

        out_str = formatter.heading(level, "".join(filter(None, [self.namespace, self.name])))
        out_str += formatter.text(self.description) + formatter.new_line()

        for key in sorted(ds.KEYS.values()):
            if key in self.children:
                out_str += formatter.heading(level + 1, key)
                full_names = [("".join(filter(None, [x.namespace, x.name])), x)
                              for x in self.children[key]]
                for (_, item) in sorted(full_names):
                    list_str = item.to_string(level + 2, formatter)
                    out_str += list_str + formatter.new_line()

        return out_str

    def add_child(self, key, child_object):
        """
        Adds a child object to the children's dict

        :param str key: The key under which the child's object should be added
        :param child_object: The child object to add. This should be another DocObject or anything
        derived from DocObject
        """
        if key in self.children:
            if not isinstance(self.children[key], list):
                raise TypeError("Not a list")
            if child_object not in self.children[key]:
                self.children[key].append(child_object)
            else:
                self.update_child(key, child_object)
        else:
            self.children[key] = [child_object]

    def update_child(self, key, child_object):
        """Updates a doc item. If description is empty, it will set it, otherwise print a warning"""

        if key in self.children:
            if child_object not in self.children[key]:
                raise KeyError("Cannot find item {} inside {}[{}]"
                               .format(child_object.name, self.name, key))
            for child in self.children[key]:
                if child.name == child_object.name:
                    if not child.description:
                        child.description = child_object.description
                    else:
                        print("WARNING: Requested to update item {} inside {}[{}]"
                              .format(child_object.name, self.name, key)
                              + " which is non-empty. This is not yet supported.")

    def get_description(self):
        """Returns the description or a hint if possible"""
        if self.description:
            return self.description

        if self.line_number:
            return self.default_description.format(self.filename, self.line_number, self.code)

        raise RuntimeError

    def set_description(self, description):
        """
        Checks wheter the given description matches the default description and sets it to this
        object's description otherwise.
        If the description matches the default description, this object's description is set empty
        and information is extracted from the given description.
        """

        if description is not None:
            match = re.search(self.default_desc_regex, " ".join(description.split()))
            if match:
                self.filename = match.group(1)
                self.line_number = match.group(2)
                self.code = match.group(3)
                return ""
        return description

    def set_name(self, ros_url, var_name):
        """
        Checks wheter the given name is in a string format meaning have leading or trailing
        quotation marks, if it has those we assume this is the actual name of the object.  If
        quotation marks are missing we assume it to be a variable in the original code and will mark
        it accordingly.
        """
        (namespace, name) = self.extract_namespace(ros_url.strip("'" + '"'))
        self.namespace = namespace
        if not var_name is None:
            self.name = name
            self.var_name = var_name
        elif ros_url and ros_url[0] == ros_url[-1] and (ros_url[0] == "'" or ros_url[0] == '"'):
            self.name = name
            self.var_name = False
        else:
            self.name = name
            self.var_name = True

    @staticmethod
    def extract_namespace(full_name):
        """
        Splits a full ros url into namespace and name
        """
        regex = r'((?P<ns>(.*\/|~)))?(?P<name>[^~,)\n]+)'
        match = re.match(regex, full_name)
        return match.group('ns'), match.group('name')

    def merge_with(self, other):
        """
        Merges in another DocObject

        The merging has a couple of principles:
         * If entries in this are empty, but filled in other, use the entry from other
         * If entries do exist in both and they are not the same, ask the user which one to pick.
         * If children don't exist in this, but in other, we should ask the user what to do.
         * If children exist in this, but not in other, we should keep them.
        """

        assert self == other, "For merging, both objects have to have the same name"

        self.merge_field(other, "description")
        self.merge_field(other, "namespace")

        for child_key in self.children:
            if child_key in other.children:
                for child in self.children[child_key]:
                    other_child = None
                    for other_c in other.children[child_key]:
                        if other_c == child:
                            other_child = other_c
                    if other_child:
                        child.merge_with(other_child)

                # Add children only existing in other
                for child in other.children[child_key]:
                    # print("Checking for child: {}".format(child.name))
                    this_child = None
                    for this_c in self.children[child_key]:
                        if this_c == child:
                            this_child = this_c
                    if this_child is None:
                        self.ask_adding(child, child_key)

    def merge_field(self, other, field_name):
        """
        Merges a single field from other with self. Applies the general rules from merge_with()
        """
        if self.__getattribute__(field_name):
            # check, whether other's description differs and ask user which one to pick
            if other.__getattribute__(field_name) != self.__getattribute__(field_name)\
                    and other.__getattribute__(field_name):
                print("{}: {} field of both versions differ.".format(self.name, field_name))
                correct_answer = False
                while not correct_answer:
                    answer = input("Please choose the version that should be included into the "
                                   "resulting documentation by typing <1> or 2. Choices:\n"
                                   "1: {}\n"
                                   "2: {}\n"
                                   "choice: "
                                   .format(self.__getattribute__(field_name),
                                           other.__getattribute__(field_name)))
                    if str(answer) == "2":
                        self.__setattr__(field_name, copy.copy(other.__getattribute__(field_name)))
                        correct_answer = True
                    elif str(answer) == "1":
                        # We leave this field untouched
                        correct_answer = True
                    else:
                        print("Illegal answer")
        elif other.__getattribute__(field_name):
            self.__setattr__(field_name, copy.copy(other.__getattribute__(field_name)))

    def ask_adding(self, child, child_key):
        """
        Ask whether a child should be added to this.
        """
        print("{}.{}: {} was not found in this.".format(
            self.name,
            child_key,
            "".join(filter(None, [child.namespace, child.name]))))
        correct_answer = False
        while not correct_answer:
            answer = input("Should this item be deleted or added to the resulting documentation?\n"
                           "1: Delete\n"
                           "2: Add\n"
                           "choice: ")
            if str(answer) == "2":
                self.add_child(child_key, child)
                correct_answer = True
            elif str(answer) == "1":
                # We leave this untouched
                correct_answer = True
            else:
                print("Illegal answer")
