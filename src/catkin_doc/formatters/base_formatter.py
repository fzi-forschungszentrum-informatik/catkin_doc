"""
This is the base formatter for DocObjects
"""

class BaseFormatter(object):
    """Abstract base class for formatting DocObjects"""
    def __init__(self):
        pass

    def heading(self, level, text):
        """
        Formats a text as a heading given a certain level

        :param int level: Level of the heading
        :param str text: Text of the heading
        :return: Formatted heading
        :rtype: str
        """

        raise NotImplementedError("This is not implemented for " + self.__class__)

    def text(self, text):
        """
        Formats plain text

        :param str text: Text
        :return: Formatted text
        :rtype: str
        """

        raise NotImplementedError("This is not implemented for " + self.__class__)
