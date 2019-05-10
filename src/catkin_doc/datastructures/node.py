"""
Node datastructure
"""

from catkin_doc.datastructures.doc_object import DocObject

class Node(DocObject):
    """Datastructure representing a node"""

    parameters_key = "Parameters"

    def add_parameter(self, parameter):
        self.add_child(self.parameters_key, parameter)

    def to_string(self, level, formatter):
        """
        Formats the object as text

        :param int level: Level of heading hierarchy
        :param formatter: Formatter to use
        :return: A formatted string for this object formatted by the given formatter
        :rtype: str
        """

        out_str = formatter.heading(level, self.name) + formatter.new_line()
        out_str += formatter.text(self.description) + formatter.new_line()

        out_str += formatter.heading(level+1, self.parameters_key)

        for item in self.children[self.parameters_key]:
            out_str += item.to_string(level+2, formatter)
            out_str += formatter.new_line()

        return out_str
