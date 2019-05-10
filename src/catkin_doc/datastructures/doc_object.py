'''
This class serves as a base class for all documentation objects
'''

class DocObject(object):
    """Base class for a doc object"""
    def __init__(self, name, description=""):
        self.name = name
        self.description = description

        self.children = dict()

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
                out_str += item.to_string(level+1, formatter)

        return out_str

    def add_child(self, key, child_object):
        """
        Adds a child object to the children's dict

        :param str key: The key under which the child's object should be added
        :param child_object: The child object to add. This should be another DocObject or anything
        derived from DocObject
        """
        if key in self.children:
            if self.children[key] != list:
                raise TypeError("Not a list")
            self.children[key].append(child_object)
        else:
            self.children[key] = [child_object]
