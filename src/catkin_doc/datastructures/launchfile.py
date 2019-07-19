"""
Launchfile datastructure
"""
import catkin_doc.datastructures as ds
from catkin_doc.datastructures.doc_object import DocObject


class LaunchFile(DocObject):

    def add_argument(self, argument):
        """Adds a argument as child"""
        self.add_child(ds.KEYS["launch_argument"], argument)

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

        if not self.description and not self.children:
            out_str += formatter.text("No arguments for this launch file found. You can add a"
                                      " description by hand, if you like.")
        for key in sorted(ds.KEYS.values()):
            if key in self.children:
                out_str += formatter.heading(level + 1, key)
                for item in sorted(self.children[key]):
                    list_str = item.to_string(level + 2, formatter)
                    out_str += formatter.as_list_item(0, list_str) + formatter.new_line()

        return out_str
