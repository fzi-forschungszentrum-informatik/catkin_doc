#!/usr/env/bin python
"""catkin_doc helps creating documentations for catkin packages"""

import catkin_doc.cmakeparser
import catkin_doc.pkghandler
import catkin_doc.nodeconverter
import catkin_doc.rstparser
import catkin_doc.nodecomparator
import sys


package_path = sys.argv[1]
file_format = sys.argv[2]
cpp_handler = catkin_doc.cmakeparser.CmakeListParser(package_path)
py_handler = catkin_doc.pkghandler.PkgHandler(package_path)
nodeconverter = catkin_doc.nodeconverter.NodeConverter()
docu_list = catkin_doc.pkghandler.PkgHandler.find_docu(package_path)

"""
This script will search for python and cpp nodes.
than for each of this nodes it will search for existing docu
if no docu is found it will ask for path to docu
than parse each docu file and compare to previously generated nodes
at last the merged nodes are written to given file format
"""

for parser in cpp_handler.parser:
    found_parser = False
    for docu in docu_list:
      if (parser.node.filename + ".rst") in docu:
          found_parser = True
          rst_parser = catkin_doc.rstparser.RstParser(docu)
          comp = catkin_doc.nodecomparator.NodeComparator(parser.node, rst_parser.node)
          nodeconverter.convert_to_file(comp.merged_node, file_format)
    if not found_parser:
        answer = raw_input("Did not found matching docu for node " + parser.node.filename + ". \n Please enter path to documentation or type 's' to skip compare and write node to docu. \n")
        if answer == "s":
            nodeconverter.convert_to_file(parser.node, file_format)
        else:
            if ".rst" in answer:
                rst_parser = catkin_doc.rstparser.RstParser(answer)
                comp = catkin_doc.nodecomparator.NodeComparator(parser.node, rst_parser.node)
                nodeconverter.convert_to_file(comp.merged_node, file_format)
            else:
                print("Inserted path does not match an rst path. Skipping compare ...")
                nodeconverter.convert_to_file(parser.node, file_format)



