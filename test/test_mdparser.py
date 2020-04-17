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

from __future__ import print_function

import os
import unittest


from catkin_doc.parsers.mdparser import MdParser
from catkin_doc.parsers.mdparser import DocSection
import catkin_doc.datastructures as ds
from catkin_doc.datastructures.doc_object import DocObject
from catkin_doc.datastructures.parameter import Parameter, LaunchArgument
from catkin_doc.datastructures.package import Package
from catkin_doc.datastructures.topic import Publisher, Subscriber
from catkin_doc.datastructures.node import Node
from catkin_doc.datastructures.action import Action, ActionClient
from catkin_doc.datastructures.service import Service, ServiceClient
from catkin_doc.datastructures.launchfile import LaunchFile


class TestMdParsing(unittest.TestCase):
    """Test basic functionality of the md parsing module"""

    def test_parsing(self):
        """Test parsing of a generated markdown file"""
        script_dir = os.path.dirname(__file__)  # <-- absolute dir the script is in
        rel_path = 'test_package/README.md'
        abs_path = os.path.join(script_dir, rel_path)

        parser = MdParser(abs_path)
        self.maxDiff = None

        self.assertEqual(parser.doc.name, 'test_package')
        self.assertEqual(parser.doc.description, 'The test_package package')

        self.assertListEqual(sorted(parser.doc.children.keys()), ['Launchfiles', 'Nodes'])
        nodes = parser.doc.children['Nodes']

        self.assertListEqual(list(nodes.children.keys()), ['test_package_node', 'python_node'])

        # This could go on forever... But this will be covered in the full-blown package test

    def test_illegal_file(self):
        """Test parsing of a generated markdown file"""
        script_dir = os.path.dirname(__file__)  # <-- absolute dir the script is in
        rel_path = 'test_package/package.xml'
        abs_path = os.path.join(script_dir, rel_path)

        with self.assertRaises(RuntimeError):
            parser = MdParser(abs_path)

    def test_parameter_parsing(self):
        """Test parsing of a parameter line"""
        md_code = r'''## ~tcp_port (default: "54321")

Port on which the remote pc (robot) publishes the interface
'''
        doc_section = DocSection(md_code.splitlines(), Parameter, level=1)

        self.assertEqual(doc_section.name, '~tcp_port')
        self.assertEqual(doc_section.description,
                         'Port on which the remote pc (robot) publishes the interface')
        self.assertEqual(doc_section.default_value, '"54321"')
        self.assertEqual(doc_section.var_name, False)

        md_code = r'''## Symbol: param_name (Required)

This is a required parameter with a symbol as name
'''
        doc_section = DocSection(md_code.splitlines(), Parameter, level=1)

        self.assertEqual(doc_section.name, 'param_name')
        self.assertEqual(doc_section.description,
                         'This is a required parameter with a symbol as name')
        self.assertEqual(doc_section.default_value, None)
        self.assertEqual(doc_section.var_name, True)

    def test_launch_argument_parsing(self):
        """Test parsing of a launch argument line"""
        md_code = r'''## file_arg (default: "$(find my_package)/config.yaml")

Fancy config file.
'''
        doc_section = DocSection(md_code.splitlines(), LaunchArgument, level=1)

        self.assertEqual(doc_section.name, 'file_arg')
        self.assertEqual(doc_section.description,
                         'Fancy config file.')
        self.assertEqual(doc_section.default_value, '"$(find my_package)/config.yaml"')


    def test_service_client_parsing(self):
        """Test parsing of a service_client line"""
        md_code = r'''## /move_base/clear_costmaps ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html))

Service client for resetting the costmaps
'''
        doc_section = DocSection(md_code.splitlines(), Parameter, level=1)

        self.assertEqual(doc_section.name, '/move_base/clear_costmaps')
        self.assertEqual(doc_section.description,
                         'Service client for resetting the costmaps')
        self.assertEqual(doc_section.type_info, 'std_srvs/Empty')
        self.assertEqual(doc_section.var_name, False)

    def test_service_server_parsing(self):
        """Test parsing of a service_server line"""
        md_code = r'''## set_enabled ([std_srvs/SetBool](http://docs.ros.org/api/std_srvs/html/srv/SetBool.html))

Service to enable and disable this component
'''
        doc_section = DocSection(md_code.splitlines(), Parameter, level=1)

        self.assertEqual(doc_section.name, 'set_enabled')
        self.assertEqual(doc_section.description,
                         'Service to enable and disable this component')
        self.assertEqual(doc_section.type_info, 'std_srvs/SetBool')
        self.assertEqual(doc_section.var_name, False)

    def test_subscriber_parsing(self):
        """Test parsing a subscriber section"""

        md_code = r'''## chatter7 ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

This is the description
'''
        doc_section = DocSection(md_code.splitlines(), Subscriber, level=1)
        self.assertEqual(doc_section.name, 'chatter7')
        self.assertEqual(doc_section.description,
                         'This is the description')
        self.assertEqual(doc_section.type_info, 'std_msgs/String')
        self.assertEqual(doc_section.var_name, False)

        md_code = r'''## chatter8 (my_own_msgs/MyType)

This is the description
'''
        doc_section = DocSection(md_code.splitlines(), Subscriber, level=1)
        self.assertEqual(doc_section.name, 'chatter8')
        self.assertEqual(doc_section.description,
                         'This is the description')
        self.assertEqual(doc_section.type_info, 'my_own_msgs/MyType')
        self.assertEqual(doc_section.var_name, False)

    def test_section_parsing(self):
        """Test parsing of section headings"""
        md_code = r'''This is some unnecessary text

# my_package
This is the package description
## Nodes
### my test node
## Launch files
# my_test_launchfile
'''
        # We should not find anything on that level
        doc_section = DocSection(md_code.splitlines(), Package, level=5)
        self.assertEqual(doc_section.name, None)

        doc_section = DocSection(md_code.splitlines(), Package, level=0)
        self.assertEqual(doc_section.name, 'my_package')

    def test_to_doc_object(self):
        """Test conversion from a DocSection to a DocObject"""
        md_code = r'''## set_enabled ([std_srvs/SetBool](http://docs.ros.org/api/std_srvs/html/srv/SetBool.html))

Service to enable and disable this component
'''
        doc_section = DocSection(md_code.splitlines(), Parameter, level=1)
        doc_object = doc_section.to_doc_object()

        self.assertEqual(doc_object.name, doc_section.name)
        self.assertEqual(doc_object.description, doc_section.description)
        self.assertEqual(doc_object.datatype, doc_section.type_info)
        self.assertEqual(doc_object.var_name, doc_section.var_name)
        self.assertIsInstance(doc_object, Parameter)

        md_code = r'''## hello ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Listener
'''
        doc_section = DocSection(md_code.splitlines(), Subscriber, level=1)
        doc_object = doc_section.to_doc_object()

        self.assertEqual(doc_object.name, doc_section.name)
        self.assertEqual(doc_object.description, doc_section.description)
        self.assertEqual(doc_object.datatype, doc_section.type_info)
        self.assertEqual(doc_object.var_name, doc_section.var_name)
        self.assertIsInstance(doc_object, Subscriber)

        md_code = r'''This is some unnecessary text

# my_package
This is the package description
## Nodes
### my test node
## Launch files
# my_test_launchfile
'''
        doc_section = DocSection(md_code.splitlines(), Package, level=0)
        doc_object = doc_section.to_doc_object()
        self.assertEqual(doc_object.name, doc_section.name)
        self.assertEqual(doc_object.description, doc_section.description)
        self.assertEqual(doc_object.var_name, doc_section.var_name)
        self.assertIsInstance(doc_object, Package)

    def test_to_string(self):
        """Test that conversion to string actually works"""
        md_code = r'''## hello ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Listener
'''
        doc_section = DocSection(md_code.splitlines(), Subscriber, level=1)
        print(str(doc_section))

        md_code = r'''This is some unnecessary text

# my_package
This is the package description
## Nodes
### my test node
## Launch files
# my_test_launchfile
'''
        doc_section = DocSection(md_code.splitlines(), Package, level=0)
        print(str(doc_section))

        md_code = r'''## ~tcp_port (default: "54321")

Port on which the remote pc (robot) publishes the interface
'''
        doc_section = DocSection(md_code.splitlines(), Parameter, level=1)
        print(str(doc_section))


if __name__ == '__main__':
    unittest.main()
