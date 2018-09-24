import os
import re
import catkin_doc.cpp
class CmakeListParser:
    __init__(self, pkg_name):
        self.pkg_name = pkg_name
        self.executables = dict()
        self.search_for_cpp_node()
        self.parser = list()
        for node in self.executables:
            self.parser.add(catkin_doc.cpp.CppParser(self.pkg_name, self.executables[node]))


def search_for_cpp_node(self):
    """
    Method which searches the CMakeLists.txt for passible node entries
    """
    if os.path.isfile(self.pkg_path + "/CMakeLists.txt"):
        with open(self.pkg_path + "/CMakeLists.txt") as filecontent:
            self.lines = filecontent.readlines()
        linenumber = 0
        while linenumber < len(self.lines):
            self.parse_project_name(linenumber)
            self.parse_executables(linenumber)
            linenumber += 1
        self.remove_not_nodes()

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

    TODO: What about one line definitions like add_executable(name file file)
    Maybe first check if regex: (add_executable\()(\S+)(((\s)(\S+))+)\) is match an if so
    Reading group3 (files) as string an cutting at whitespaces + check if directories
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
                if os.path.isfile(self.pkg_path + "/" + cpp_file):
                  cpp_files.append(self.pkg_path + "/" + cpp_file)
                elif os.path.isdir(self.pkg_path + "/" + cpp_file):
                    for filename in os.listdir(self.pkg_path):
                        cpp_files.append(self.pkg_path + "/" + filename)

            linenumber += 1
        self.executables[exec_name] = cpp_files


def remove_not_nodes(self):
    """
    Method to remove executables which are not ros nodes.
    This is done by checking if one of the corresponding files contains ros::init()
    """
    for key in self.executables:
        node = False
        for file in self.executables[key]:
            content = open(file, "r")
            if "ros::init" in content.read():
                node = True
        if not node:
            self.executables.pop(key)
