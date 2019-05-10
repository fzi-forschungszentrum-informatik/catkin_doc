"""
DocObject formatter for markdown
"""

from catkin_doc.formatters.base_formatter import BaseFormatter
import markdown_strings

class MarkdownFormatter(BaseFormatter):
    """Formats to markdown"""
    def heading(self, level, text):
        return markdown_strings.header(text, level) + "\n"

    def text(self, text):
        return text + "\n"


