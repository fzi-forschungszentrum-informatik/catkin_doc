"""
This is the base formatter for DocObjects
"""


class BaseFormatter(object):
    """Abstract base class for formatting DocObjects"""
    def heading(self, level, text):
        """
        Formats a text as a heading given a certain level

        :param int level: Level of the heading
        :param str text: Text of the heading
        :return: Formatted heading with appended newline
        :rtype: str
        """

        raise NotImplementedError(
            "heading(): This is not implemented for {}" .format(self.__class__))

    def text(self, text, newline=True):
        """
        Formats plain text

        :param str text: Text
        :param bool newline: Should a newline be appended at the end?
        :return: Formatted text without a new line
        :rtype: str
        """

        raise NotImplementedError("text(): This is not implemented for {}" .format(self.__class__))

    def bold(self, text):
        """
        Formats text as bold

        :param str text: Text
        :return: Formatted text without a new line
        :rtype: str
        """

        raise NotImplementedError("text(): This is not implemented for {}" .format(self.__class__))

    def new_line(self):
        """
        Formats a new line

        :return: Formatted text
        :rtype: str
        """

        raise NotImplementedError("new_line(): This is not implemented for {}"
                                  .format(self.__class__.__name__))
