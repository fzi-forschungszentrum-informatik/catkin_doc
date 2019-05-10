'''
This class serves as a base class for all documentation objects
'''


class DocObject(object):
    """Base class for a doc object"""

    def __init__(self, name, description=""):
        self.name = name
        self.description = description

        self.filename = ""
        self.line_number = None
        self.code = ""

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
                        print "WARNING: Requested to update item {} inside {}[{}]"\
                              .format(child_object.name, self.name, key)\
                              + " which is non-empty. This is not yet supported."

    def get_description(self):
        """Returns the description or a hint if possible"""
        if self.description:
            return self.description

        if self.line_number:
            return "Please add description. See {} line number: {}\n    Code: {}"\
                .format(self.filename, self.line_number, self.code)
