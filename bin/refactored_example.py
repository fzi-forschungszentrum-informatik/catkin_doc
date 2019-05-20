#!/usr/bin/env python

import catkin_doc.datastructures as ds
from catkin_doc.datastructures.package import Package

from catkin_doc.formatters.markdown_formatter import MarkdownFormatter

from catkin_doc.parsers.pythonparser import PythonParser
from catkin_doc.parsers.cppparser import CppParser
from catkin_doc.parsers.mdparser import MdParser
from catkin_doc.parsers.launchparser import LaunchParser


def main():
    main_object = Package("my_package")

    # child_object = DocObject("component1", "This is the first component")
    # main_object.add_child("components", child_object)

    # node_object = Node("example_node", "This node doesn't do anything special")
    # parameter1 = Parameter("~my_parameter", description="Very fancy parameter")
    # node_object.add_parameter(parameter1)
    # parameter2 = Parameter("~my_def_parameter", description="Another fancy parameter",
                           # default_value=0.5)
    # node_object.add_parameter(parameter2)
    # # main_object.add_node(node_object)

    print("Parsing launch files")
    launchparser  = LaunchParser('test.launch')
    main_object.add_launchfile(launchparser.launchfile)

    print("Parsing source code")
    pyparser = PythonParser("test_node.py")
    main_object.add_node(pyparser.node)

    # cppparser = CppParser("test_cpp_node",["test.cpp"])
    # main_object.add_node(cppparser.node)

    formatter = MarkdownFormatter()
    markdown_string = main_object.to_string(1, formatter)


    with open("output.md", "w") as f:
        f.write(markdown_string)

    print("Parsing existing documentation")
    mdparser = MdParser(filename="output.md")

    # print("-----")
    #print(mdparser.doc)
    # print("-----")
    md_docobj = mdparser.doc.to_doc_object()

    markdown_string = md_docobj.to_string(1, formatter)
    # print (markdown_string)
    with open("output_new.md", "w") as f:
         f.write(markdown_string)

    print("Merging documentations")
    main_object.merge_with(md_docobj)
    markdown_string = main_object.to_string(1, formatter)
    with open("output_merged.md", "w") as f:
        f.write(markdown_string)


if __name__ == "__main__":
    main()
