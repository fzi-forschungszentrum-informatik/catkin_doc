"""
Topic datastructure
"""

from catkin_doc.datastructures.doc_object import DocObject


class Topic(DocObject):
    """Datastructure representing a subscribed or advertised topic"""

    def __init__(self, name, description="", datatype=""):
        super(Topic, self).__init__(name, description)
        self.datatype = datatype

    def to_string(self, level, formatter):
        """
        Formats the object as text

        :param int level: Level of heading hierarchy
        :param formatter: Formatter to use
        :return: A formatted string for this object formatted by the given formatter
        :rtype: str
        """

        out_str = formatter.bold(self.name)
        out_str += formatter.text(" ({})".format(self.datatype))
        out_str += formatter.new_line()

        out_str += formatter.text(self.get_description())

        return out_str
