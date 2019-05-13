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

    def as_list_item(self, level, formatted_text):
        """Turns the given text into a list item. Note: The text has to be formatted in advance,
        this is not done in this function.

        :param int level: List indentation level (lowest is 0)
        :param str text: Preformatted text
        :return: Formatted text
        :rtype: str
        """
        raise NotImplementedError(
            "as_list_item(): This is not implemented for {}" .format(self.__class__))

    def link(self, url, text=""):
        """Formats a link properly

        :param str url: The link's URL
        :param str text: The text shown for the link. If empty, the url will be shown instead
        :return: Formatted text
        :rtype: str
        """
        raise NotImplementedError(
            "link(): This is not implemented for {}" .format(self.__class__))
