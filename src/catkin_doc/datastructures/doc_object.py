'''
This class serves as a base class for all documentation objects
'''
import re


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
        return self.name == other.name

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
            print self.default_desc_regex
            print " ".join(description.split())
            print "''" + match.group(1) + "''"
            print "''" + match.group(2) + "''"
            print "''" + match.group(3) + "''"
            self.filename = match.group(1)
            self.line_number = match.group(2)
            self.code = match.group(3)
            return ""
        return description
