"""
DocObject formatter for markdown
"""

from catkin_doc.formatters.base_formatter import BaseFormatter
import markdown_strings

class MarkdownFormatter(BaseFormatter):
    """Formats to markdown"""
    def heading(self, level, text):
        return markdown_strings.header(text, level) + "\n"

    def text(self, text, newline=True):
        if newline:
            return "{}\n".format(text)
        else:
            return "{}".format(text)

    def new_line(self):
        return "\n"

    def bold(self, text):
        return markdown_strings.bold(text)

    def as_list_item(self, level, formatted_text):
        new_newline = "\n"+level*"  " + "    "
        new_text =  new_newline.join(formatted_text.split("\n"))
        return " ".join([level * "  ", "*", new_text])
