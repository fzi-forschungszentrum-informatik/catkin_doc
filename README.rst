The catkin_doc project
=======================

Generally this package generates some documentation for ros nodes.
For each ros node declared in the package python nodes as well as cpp nodes the project generates a documentation file.
The file can be either mark-down or restructuredtext depending on your input.

How to use
___________
Just type in your console for restruckturedtext docu:
::
    python bin/catkin_doc "/path/to/your/package" "rst"
    
or for markdown docu:
::
    python bin/catkin_doc "/path/to/your/package" "md"