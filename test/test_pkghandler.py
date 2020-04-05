#!/usr/bin/env python
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


import os
import unittest

from catkin_doc.pkghandler import PkgHandler

class TestMdParsing(unittest.TestCase):
    """Test basic functionality of the md parsing module"""

    def test_check_doc(self):
        """Tests the fucntionality to find existing documentation"""
        script_dir = os.path.dirname(__file__)  # <-- absolute dir the script is in
        rel_path = 'test_package'
        pkg_path = os.path.join(script_dir, rel_path)

        doc_object = PkgHandler.find_existing_docu(pkg_path, "README.md")
        self.assertEqual(doc_object.name, "test_package")

        doc_object = PkgHandler.find_existing_docu(pkg_path, "ROS_API.md")
        self.assertIsNone(doc_object)

    def test_is_ros_node(self):
        """Tests the check_if_ros_node function"""
        script_dir = os.path.dirname(__file__)  # <-- absolute dir the script is in
        rel_path = 'test_package/scripts/non_rospy'
        abs_path = os.path.join(script_dir, rel_path)
        is_ros_node = PkgHandler.check_if_ros_node(abs_path)
        self.assertFalse(is_ros_node)

        rel_path = 'test_package/scripts/python_node'
        abs_path = os.path.join(script_dir, rel_path)
        is_ros_node = PkgHandler.check_if_ros_node(abs_path)
        self.assertTrue(is_ros_node)

    def test_parsing(self):
        """Tests parsing of the test_package"""
        script_dir = os.path.dirname(__file__)  # <-- absolute dir the script is in
        rel_path = 'test_package'
        pkg_path = os.path.join(script_dir, rel_path)

        pkg_handler = PkgHandler(pkg_path, "README.md")

        # Information from package.xml
        self.assertEqual(pkg_handler.doc.name, "test_package")
        self.assertEqual(pkg_handler.doc.description, "The test_package package")

        # Find launchfiles
        expected_launchfiles = [
            os.path.join(pkg_path, 'launch/test_package_node_with_arg.launch'),
            os.path.join(pkg_path, 'launch/test_package_node.launch')
        ]
        self.assertListEqual(pkg_handler.launch_files, expected_launchfiles)

        # Find python nodes
        expected_python_nodes = [os.path.join(pkg_path,'scripts/python_node')]
        self.assertListEqual(pkg_handler.python_nodes, expected_python_nodes)



if __name__ == '__main__':
    unittest.main()
