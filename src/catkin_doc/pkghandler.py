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

"""
Toplevel instance for parsing a package and its contents.
"""

from __future__ import print_function
import os


import xml.etree.ElementTree as ET
import magic

from catkin_doc.datastructures.package import Package

from catkin_doc.parsers.cmakeparser import CMakeParser
from catkin_doc.parsers.pythonparser import PythonParser
from catkin_doc.parsers.cppparser import CppParser
from catkin_doc.parsers.mdparser import MdParser
from catkin_doc.parsers.launchparser import LaunchParser


class PkgHandler(object):
    """
    Toplevel instance for parsing a package and its contents.
    """

    def __init__(self, pkg_path):
        self.pkg_path = os.path.abspath(pkg_path)

        print (self.pkg_path)
        # Get package information from package.xml
        package_name = self.parse_package_xml("name")
        description = self.parse_package_xml("description")

        # Create package's DocObject
        self.doc = Package(package_name, description=description)

        # Find and parse launch files
        self.launch_files = list()
        self.find_launchfiles(self.pkg_path)
        for file in self.launch_files:
            launchparser = LaunchParser(file, self.pkg_path)
            self.doc.add_launchfile(launchparser.launchfile)

        # Find and parse c++ nodes
        self._cmake_handler = CMakeParser(self.pkg_path)
        for node in self._cmake_handler.executables:
            cppparser = CppParser(node, self._cmake_handler.executables[node])
            self.doc.add_node(cppparser.node)

        # Find and parse python nodes
        self.python_nodes = list()
        self.find_python_nodes(self.pkg_path)
        for node in self.python_nodes:
            pyparser = PythonParser(node)
            self.doc.add_node(pyparser.node)

        # find existing documentation and merge it
        existing_doc = self.find_existing_docu(self.pkg_path)
        if existing_doc:
            self.doc.merge_with(existing_doc)

    def parse_package_xml(self, field):
        """
        Parses a field out of the  package.xml
        """

        tree = ET.parse(self.pkg_path + '/package.xml').getroot()
        return tree.find(field).text.strip()

    def find_python_nodes(self, pkg_path):
        """
        Method which searches through a whole package for python ros nodes
        Currently prints filenames of files which are probably python ros nodes
        """

        for filename in os.listdir(pkg_path):
            if os.path.isdir(pkg_path + "/" + filename):
                self.find_python_nodes(pkg_path + "/" + filename)
            elif os.path.isfile(pkg_path + "/" + filename):
                filetype = magic.from_file(pkg_path + "/" + filename)
                if ("python" in filetype) | ("Python" in filetype):
                    if PkgHandler.check_if_ros_node(pkg_path + "/" + filename):
                        print("Adding node " + filename)
                        self.python_nodes.append(pkg_path + "/" + filename)

    def find_launchfiles(self, pkg_path):
        """
        Method which searches through a whole package for launchfiles
        """

        for filename in os.listdir(pkg_path):
            if os.path.isdir(pkg_path + "/" + filename):
                self.find_launchfiles((pkg_path + "/" + filename))
            elif os.path.isfile(pkg_path + "/" + filename):
                filetype = magic.from_file(pkg_path + "/" + filename)
                if ("launch" in filename) and ("XML" in filetype):
                    self.launch_files.append(pkg_path + "/" + filename)

    @staticmethod
    def check_if_ros_node(filename):
        """
        Method which checks if file contains the string "rospy.init_node"
        as this is a good clue that this file may be a python ros node.
        Returns True if stri8ng is containes False otherwise
        """
        filecontent = open(filename, "r")
        if "rospy.init_node" in filecontent.read():
            print(filename)
            return True
        return False

    @staticmethod
    def find_existing_docu(pkg_path):
        """
        Finds and parses existing documentation. Currently, only README.md at top level is searched
        """
        docufile = None
        for filename in os.listdir(pkg_path):
            if "README.md" in filename:
                docufile = pkg_path + "/" + filename
                break

        if not docufile:
            print("Didn't find an existing documentation")
            return None

        mdparser = MdParser(filename=docufile)

        return mdparser.doc.to_doc_object()
