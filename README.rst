The catkin_doc project
=======================

Generally this package generates some documentation for ros nodes.
For each ros node declared in the package python nodes as well as cpp nodes the project generates a documentation file.
The file can be either mark-down or restructuredtext depending on your input.

How to use
___________
At the moment there is no script to run which solves all your problem.
But you can already generate documentation by using a little bit of python.
First of all you have to decide wheter you have python or cpp nodes you want docu for.

Cpp-Nodes
~~~~~~~~~
To get all cpp-nodes from your package you have to create a cmakeparser.
So import the catkin_doc.cmakeparser module and then create a parser for your package by
::
    cmakeparser = catkin_doc.cmakeparser.CmakeListParser("path\to\your\package")
    
The cmakeparser will parse the CmakeList.txt and will create a cpp-parser for each executable which seems to be a ros node.
To create a output file for your nodes you have to use the nodeconverter.
First import the module and then create a nodeconverter by 
::
    conv = catkin_doc.nodeconverter.NodeConverter()


To convert a node to a file you use the function convert_to_file so for example
::
    conv.convert_to_file(cmakeparser.parser[1].node, "rst")
    
To see which parser you want to use you can look at the list parser.executables there you can see the nodenames in the same order as in the parser list.
