"""
docobject formatter for markdown
"""

import markdown_strings
from catkin_doc.formatters.base_formatter import BaseFormatter


class MarkdownFormatter(BaseFormatter):
    """Formats to markdown"""

    def heading(self, level, text):
        return markdown_strings.header(text, level) + "\n"

    def text(self, text, newline=True):
        if newline:
            return "{}\n".format(text)
        return "{}".format(text)

    def new_line(self):
        return "\n"

    def bold(self, text):
        return markdown_strings.bold(text)

    def as_list_item(self, level, formatted_text):
        new_newline = "\n" + level * "  " + "    "
        new_text = new_newline.join(formatted_text.split("\n"))
        result_lines = [line.rstrip() for line in new_text.split("\n")]
        result = " ".join([level * "  ", "*", "\n".join(result_lines)])
        # print result.split("\n")

        return result

    def link(self, url, text=""):
        return markdown_strings.link(text, url)
