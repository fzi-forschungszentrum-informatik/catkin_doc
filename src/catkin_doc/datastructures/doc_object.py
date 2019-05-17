'''
This class serves as a base class for all documentation objects
'''
from __future__ import print_function

import re
import copy


class DocObject(object):
    """Base class for a doc object"""

    def __init__(self, name, description=""):
        self.default_description = "Please add description. See {} line number: {}\n\tCode: {}"
        self.default_desc_regex = "\s+".join(
            self.default_description.format("(.*)", "(\d+)", "(.*)").split())

        self.name = name
        # This is neede for the default description
        self.filename = ""
        self.line_number = None
        self.code = ""
        self.description = self.set_description(description)

        self.children = dict()

    def __eq__(self, other):
        return self.name == other.name and type(self) == type(other)

    def to_string(self, level, formatter):
        """
        Formats the object as markdown text

        :param int level: Level of heading hierarchy
        :param formatter: Formatter to use
        :return: A formatted string for this object formatted by the given formatter
        :rtype: str
        """

        out_str = formatter.heading(level, self.name)
        out_str += formatter.text(self.description)

        for key in self.children:
            for item in self.children[key]:
                out_str += item.to_string(level + 1, formatter)

        return out_str

    def add_child(self, key, child_object):
        """
        Adds a child object to the children's dict

        :param str key: The key under which the child's object should be added
        :param child_object: The child object to add. This should be another DocObject or anything
        derived from DocObject
        """
        if key in self.children:
            if type(self.children[key]) != list:
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

    def set_description(self, description):
        """
        Checks wheter the given description matches the default description and sets it to this
        object's description otherwise.
        If the description matches the default description, this object's description is set empty
        and information is extracted from the given description.
        """

        match = re.search(self.default_desc_regex, " ".join(description.split()))
        if match:
            self.filename = match.group(1)
            self.line_number = match.group(2)
            self.code = match.group(3)
            return ""
        return description

    def merge_with(self, other):
        """
        Merges in another DocObject

        The merging has a couple of principles:
         * If entries in this are empty, but filled in other, use the entry from other
         * If entries do exist in both and they are not the same, ask the user which one to pick.
         * If children don't exist in this, but in other, we should ignore those entries.
         * If children exist in this, but not in other, we should keep them.
        """

        assert self == other, "For merging, both objects have to have the same name"

        self.merge_field(other, "description")

        for child_key in self.children:
            if child_key in other.children:
                for child in self.children[child_key]:
                    other_child = None
                    for other_c in other.children[child_key]:
                        if other_c == child:
                            other_child = other_c
                    if other_child:
                        child.merge_with(other_child)

    def merge_field(self, other, field_name):
        """
        Merges a single field from other with self. Applies the general rules from merge_with()
        """
        if self.__getattribute__(field_name):
            # check, whether other's description differs and ask user which one to pick
            if other.__getattribute__(field_name) != self.__getattribute__(field_name)\
                    and other.__getattribute__(field_name):
                print("{}: Description of both versions differ.".format(self.name))
                answer = raw_input("Please choose the version that should be included into the"
                                   "resulting documentation by typing <1> or 2. Choices:\n"
                                   "1: {}\n"
                                   "2: {}\n"
                                   "choice: "
                                   .format(self.__getattribute__(field_name),
                                           other.__getattribute__(field_name)))
                if str(answer) == "2":
                    self.__setattr__(field_name, copy.copy(other.__getattribute__(field_name)))
        elif other.__getattribute__(field_name):
            self.__setattr__(field_name, copy.copy(other.__getattribute__(field_name)))
