"""
Parameter datastructure
"""

from catkin_doc.datastructures.doc_object import DocObject


class Parameter(DocObject):
    """Datastructure representing a node's parameter"""

    def __init__(self, name, description="", datatype="", default_value=""):
        super(Parameter, self).__init__(name, description)
        self.datatype = datatype
        self.default_value = default_value

    def to_string(self, level, formatter):
        """
        Formats the object as text

        :param int level: Level of heading hierarchy
        :param formatter: Formatter to use
        :return: A formatted string for this object formatted by the given formatter
        :rtype: str
        """

        out_str = "\"" + formatter.bold(self.name) + "\""
        if self.default_value:
            default_formatted = self.default_value
            if isinstance(self.default_value, str):
                default_formatted = "\"{}\"".format(self.default_value)
            out_str += formatter.text(" (default: {})".format(default_formatted))
        else:
            out_str += formatter.new_line()
        out_str += formatter.new_line()

        out_str += formatter.text(self.get_description())

        return out_str


class LaunchArgument(Parameter):
    """Datastructure representing argument of a launchfile"""

    def __init__(self, name, description="", datatype="", default_value=""):
        super(LaunchArgument, self).__init__(name, description, datatype, default_value)
        self.default_description = "Please add description. See file \"{}\"."
        self.default_desc_regex = "\s+".join(
            self.default_description.format("(.*)", "(.*)").split())

    def get_description(self):
        """Returns the description or a hint if possible"""
        if self.description:
            return self.description

        if self.line_number:
            return self.default_description.format(self.filename, self.code)

        raise RuntimeError
