
import os
import re
import magic
import catkin_doc.python

class PkgHandler:
    def __init__(self, pkg_path):
        self.executables = list()
        self.parser = list()
        self.project_name = None
        self.pkg_path = pkg_path
        self.search_for_python(self.pkg_path)
        self.create_parser()



    def search_for_python(self, pkg_path):
        """
        Method which searches through a whole package for python ros nodes
        Currently prints filenames of files which are probably python ros nodes
        """
        for filename in os.listdir(pkg_path):
            if os.path.isdir(self.pkg_path + "/" + filename):
                self.search_for_python(pkg_path + "/" + filename)
            elif os.path.isfile(pkg_path + "/" + filename):
                filetype = magic.from_file(pkg_path + "/" + filename)
                if ("python" in filetype) | ("Python" in filetype):
                    if PkgHandler.check_if_ros_node(pkg_path + "/" + filename):
                        executables.add(pkg_path + "/" + filename)

    @staticmethod
    def check_if_ros_node(filename):
        """
        Method which checks if file contains the string "rospy.init_node"
        as this is a good clue that this file may be a python ros node.
        Returns True if stri8ng is containes False otherwise
        """
        file = open(filename, "r")
        if "rospy.init_node" in file.read():
            return True
        return False

    def create_parser(self):
        for file in self.executables:
            parser.add(catkin_doc.python.PythonParser(file))





