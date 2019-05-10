#!/usr/bin/env python

from catkin_doc.datastructures.doc_object import DocObject
from catkin_doc.datastructures.node import Node
from catkin_doc.datastructures.package import Package
from catkin_doc.datastructures.parameter import Parameter
from catkin_doc.formatters.markdown_formatter import MarkdownFormatter

from catkin_doc.parsers.pythonparser import PythonParser
from catkin_doc.parsers.mdparser import MdParser


def main():
    main_object = Package("")

    # child_object = DocObject("component1", "This is the first component")
    # main_object.add_child("components", child_object)

    # node_object = Node("example_node", "This node doesn't do anything special")
    # parameter1 = Parameter("~my_parameter", description="Very fancy parameter")
    # node_object.add_parameter(parameter1)
    # parameter2 = Parameter("~my_def_parameter", description="Another fancy parameter",
                           # default_value=0.5)
    # node_object.add_parameter(parameter2)
    # # main_object.add_node(node_object)

    # pyparser = PythonParser("test_node.py")
    # main_object.add_node(pyparser.node)

    formatter = MarkdownFormatter()
    # markdown_string = main_object.to_string(1, formatter)


    # with open("output.md", "w") as f:
        # f.write(markdown_string)

    mdparser = MdParser(filename="output.md")
    markdown_string = mdparser.doc_object.to_string(1, formatter)

    print markdown_string


if __name__ == "__main__":
    main()
