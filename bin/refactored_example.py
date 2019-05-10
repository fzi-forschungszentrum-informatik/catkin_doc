#!/usr/bin/env python

from catkin_doc.datastructures.doc_object import DocObject
from catkin_doc.formatters.markdown_formatter import MarkdownFormatter

def main():
    main_object = DocObject("my_package", "This is some really nice package")
    child_object = DocObject("component1", "This is the first component")

    main_object.add_child("components", child_object)

    formatter = MarkdownFormatter()
    markdown_string = main_object.to_string(1, formatter)

    print markdown_string

if __name__ == "__main__":
    main()
