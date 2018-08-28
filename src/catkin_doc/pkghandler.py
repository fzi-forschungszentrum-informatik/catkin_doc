
import os
import re
import magic

class PkgHandler:
    def __init__(self, pkg_path):
        self.lines = None
        self.executables = dict()
        self.project_name = None
        self.pkg_path = pkg_path
        pass



    def search_for_python(self):
        """
        Method which searches through a whole package for python ros nodes
        Currently prints filenames of files which are probably python ros nodes
        """
        for filename in os.listdir(self.pkg_path):
            if os.path.isdir(self.pkg_path + "/" + filename):
                PkgHandler.search_for_python(self.pkg_path + "/" + filename)
            elif os.path.isfile(self.pkg_path + "/" + filename):
                filetype = magic.from_file(self.pkg_path + "/" + filename)
                if ("python" in filetype) | ("Python" in filetype):
                    if PkgHandler.check_if_ros_node(self.pkg_path + "/" + filename):
                        print(self.pkg_path + "/" + filename)

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






