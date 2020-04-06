# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright (c) 2019, FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions
#    and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of
#    conditions and the following disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# -- END LICENSE BLOCK ------------------------------------------------

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

        return level * "=" + text + level * "=" + self.new_line()

    def text(self, text, newline=True):
        """
        Formats plain text

        :param str text: Text
        :param bool newline: Should a newline be appended at the end?
        :return: Formatted text without a new line
        :rtype: str
        """

        if newline:
            return text + "\n"
        else:
            return text

    def bold(self, text):
        """
        Formats text as bold

        :param str text: Text
        :return: Formatted text without a new line
        :rtype: str
        """

        return text

    def new_line(self):
        """
        Formats a new line

        :return: Formatted text
        :rtype: str
        """

        return "\n"

    def as_list_item(self, level, formatted_text):
        """Turns the given text into a list item. Note: The text has to be formatted in advance,
        this is not done in this function.

        :param int level: List indentation level (lowest is 0)
        :param str text: Preformatted text
        :return: Formatted text
        :rtype: str
        """
        return level * " " + "* " + formatted_text

    def link(self, url, text=""):
        """Formats a link properly

        :param str url: The link's URL
        :param str text: The text shown for the link. If empty, the url will be shown instead
        :return: Formatted text
        :rtype: str
        """
        if text:
            return text
        return url
