The catkin_doc project
=======================

Generally this package generates some documentation for ros nodes.
For each ros node declared in the package python nodes as well as cpp nodes the project generates a documentation file.
The file can be either mark-down or restructuredtext depending on your input.

How to use
___________
Please consider that the generated files are currently put in the directory from which you envoke the script.

How to create new documentation for a package:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Just type in your console for restruckturedtext docu:
::
    python bin/catkin_doc "/path/to/your/package" "rst"
    
or for markdown docu:
::
    python bin/catkin_doc "/path/to/your/package" "md"
    
How to update existing documentation:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
At the moment just possible for rst docu. 
Type:
::
    python bin/update_catkin_doc.py "/path/to/your/package" "rst"