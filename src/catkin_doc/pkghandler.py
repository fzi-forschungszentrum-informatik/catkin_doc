
import os
import magic

class PkgHandler:
    def __init__(self):
        pass


    @staticmethod
    def search_for_python(pkg_name):
        """
        Method which searches through a whole package for python ros nodes
        Currently prints filenames of files which are probably python ros nodes
        """
        for filename in os.listdir(pkg_name):
            if os.path.isdir(pkg_name + "/" + filename):
                PkgHandler.search_for_python(pkg_name + "/" + filename)
            elif os.path.isfile(pkg_name + "/" + filename):
              filetype = magic.from_file(pkg_name + "/" + filename)
              if ("python" in filetype) | ("Python" in filetype):
                  if PkgHandler.check_if_ros_node(pkg_name + "/" + filename):
                      print(pkg_name + "/" + filename)

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




