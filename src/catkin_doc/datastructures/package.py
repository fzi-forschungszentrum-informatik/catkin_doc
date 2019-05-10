"""
Package datastructure
"""

from catkin_doc.datastructures.doc_object import DocObject


class Package(DocObject):
    """Datastructure representing a package"""

    nodes_key = "Nodes"

    def add_node(self, node):
        self.add_child(self.nodes_key, node)

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

        out_str += formatter.heading(level + 1, self.nodes_key)

        for item in self.children[self.nodes_key]:
            out_str += item.to_string(level + 2, formatter)

        return out_str
