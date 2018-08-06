
import os
import re
import magic

class PkgHandler:
    def __init__(self):
        self.lines = None
        self.executables = dict()
        self.project_name = None
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


    def search_for_cpp_node(self, pkg_name):
        """
        Method which searches the CMakeLists.txt for passible node entries
        """
        if os.path.isfile(pkg_name + "/CMakeLists.txt"):
            with open(pkg_name + "/CMakeLists.txt") as filecontent:
                self.lines = filecontent.readlines()
            linenumber = 0
            while linenumber < len(self.lines):
                self.parse_project_name(linenumber)
                self.parse_executables(linenumber)
                linenumber += 1
            self.remove_not_nodes(pkg_name)

    def parse_project_name(self, linenumber):
        """
        Method to parse the project name from CMAkeLists.txt as it may be needed to replace Params later on
        """
        match = re.search("(project\()(\S+)(\))", self.lines[linenumber])
        if match:
            self.project_name = str(match.group(2))

    def parse_executables(self, linenumber):
        """
        Method to parse add_executable entries.
        Skipping entries containing "#" as they are hopefully a comment
        """
        if "#" in self.lines[linenumber]:
            return
        match = re.search("(add_executable\()(\S+)", self.lines[linenumber])
        if match:
            exec_name = str(match.group(2))
            if "${PROJECT_NAME}" in exec_name:
                exec_name = exec_name.replace("${PROJECT_NAME}", self.project_name)
            cpp_files = list()
            linenumber += 1
            while not(")" in self.lines[linenumber]):
                match_files = re.search("(\S+)", self.lines[linenumber])
                if match_files:
                    cpp_file = str(match_files.group(1))
                    if "${PROJECT_NAME}" in cpp_file:
                        cpp_file = cpp_file.replace("${PROJECT_NAME}", self.project_name)
                    cpp_files.append(cpp_file)
                linenumber += 1
            self.executables[exec_name] = cpp_files


    def remove_not_nodes(self, pkg_name):
        """
        Method to remove executables which are not ros nodes.
        This is done by checking if one of the corresponding files contains ros::init()
        """
        for key in self.executables:
            node = False
            for file in self.executables[key]:
                content = open(pkg_name + "/" + file, "r")
                if "ros::init" in content.read():
                    node = True
            if not node:
                self.executables.pop(key)




