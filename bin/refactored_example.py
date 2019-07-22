#!/usr/bin/env python
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

    # print("Parsing existing documentation")
    # mdparser = MdParser(filename="output.md")

    # print("-----")
    #print(mdparser.doc)
    # print("-----")
    # md_docobj = mdparser.doc.to_doc_object()

    markdown_string = md_docobj.to_string(1, formatter)
    # print (markdown_string)
    with open("output_new.md", "w") as f:
         f.write(markdown_string)

    # print("Merging documentations")
    # main_object.merge_with(md_docobj)
    # markdown_string = main_object.to_string(1, formatter)
    # with open("output_merged.md", "w") as f:
        # f.write(markdown_string)


if __name__ == "__main__":
    main()
