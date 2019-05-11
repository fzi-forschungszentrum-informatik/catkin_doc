"""
Package datastructure
"""

import catkin_doc.datastructures as ds

class Package(ds.DocObject):
    """Datastructure representing a package"""

    def add_node(self, node):
        self.add_child(ds.KEYS[ds.Node], node)

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

        if ds.KEYS[ds.Node] in self.children:
            out_str += formatter.heading(level + 1, ds.KEYS[ds.Node])

            for item in self.children[ds.KEYS[ds.Node]]:
                out_str += item.to_string(level + 2, formatter)

        return out_str
