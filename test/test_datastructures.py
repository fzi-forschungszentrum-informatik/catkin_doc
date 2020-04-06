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
Testing datastructures factory
"""
import unittest

import catkin_doc.datastructures as ds
from catkin_doc.datastructures.doc_object import DocObject
from catkin_doc.datastructures.node import Node
from catkin_doc.datastructures.parameter import Parameter, LaunchArgument
from catkin_doc.datastructures.package import Package
from catkin_doc.datastructures.service import Service, ServiceClient
from catkin_doc.datastructures.action import Action, ActionClient
from catkin_doc.datastructures.topic import Subscriber, Publisher
from catkin_doc.datastructures.launchfile import LaunchFile
from catkin_doc.formatters.base_formatter import BaseFormatter

class TestPython(unittest.TestCase):
    """Test factory for datastructures"""

    def test_object_from_key(self):
        """Tests creating an object from a key entry"""

        result_cls = ds.create_doc_object(ds.KEYS["package"])
        self.assertEqual(result_cls, ds.package.Package)

        result_cls = ds.create_doc_object(ds.KEYS["parameter"])
        self.assertEqual(result_cls, ds.parameter.Parameter)

        result_cls = ds.create_doc_object(ds.KEYS["service"])
        self.assertEqual(result_cls, ds.service.Service)

        result_cls = ds.create_doc_object(ds.KEYS["service_client"])
        self.assertEqual(result_cls, ds.service.ServiceClient)

        result_cls = ds.create_doc_object(ds.KEYS["publisher"])
        self.assertEqual(result_cls, ds.topic.Publisher)

        result_cls = ds.create_doc_object(ds.KEYS["subscriber"])
        self.assertEqual(result_cls, ds.topic.Subscriber)

        result_cls = ds.create_doc_object(ds.KEYS["action"])
        self.assertEqual(result_cls, ds.action.Action)

        result_cls = ds.create_doc_object(ds.KEYS["action_client"])
        self.assertEqual(result_cls, ds.action.ActionClient)

        result_cls = ds.create_doc_object(ds.KEYS["launchfile"])
        self.assertEqual(result_cls, ds.launchfile.LaunchFile)

        result_cls = ds.create_doc_object(ds.KEYS["launch_argument"])
        self.assertEqual(result_cls, ds.parameter.LaunchArgument)

        result_cls = ds.create_doc_object(ds.KEYS["node"])
        self.assertEqual(result_cls, ds.node.Node)

        result_cls = ds.create_doc_object("illegal")
        self.assertEqual(result_cls, ds.doc_object.DocObject)

    def test_identifier(self):
        """Tests getting identifiers for DocObject types"""
        self.assertEqual(ds.get_identifier_for_type(Package), "package")
        self.assertEqual(ds.get_identifier_for_type(Node), "node")
        self.assertEqual(ds.get_identifier_for_type(Parameter), "parameter")
        self.assertEqual(ds.get_identifier_for_type(Service), "service")
        self.assertEqual(ds.get_identifier_for_type(ServiceClient), "service_client")
        self.assertEqual(ds.get_identifier_for_type(Subscriber), "subscriber")
        self.assertEqual(ds.get_identifier_for_type(Publisher), "publisher")
        self.assertEqual(ds.get_identifier_for_type(Action), "action")
        self.assertEqual(ds.get_identifier_for_type(ActionClient), "action_client")
        self.assertEqual(ds.get_identifier_for_type(LaunchArgument), "launch_argument")
        self.assertEqual(ds.get_identifier_for_type(LaunchFile), "launchfile")
        self.assertEqual(ds.get_identifier_for_type(Node), "node")
        self.assertEqual(ds.get_identifier_for_type(str), "unknown")

    def test_doc_object_str(self):
        """Tests string formatting of a DocObject"""
        doc_object = DocObject(
            name="test_object", description="test_description", var_name=False)
        doc_object.add_child(ds.KEYS['publisher'], Publisher(
            name="my_pub", description="My Publisher"))

        print(doc_object)

        expected_string = """test_object
  Published topics
    my_pub

"""
        assert expected_string == str(doc_object)


    def test_doc_object_compare(self):
        """Tests comparison operators for DocObjects"""
        doc_object_1 = DocObject(
            name="abc", description="test_description", var_name=False)
        doc_object_2 = DocObject(
            name="def", description="test_description", var_name=False)

        self.assertLess(doc_object_1, doc_object_2)
        self.assertLessEqual(doc_object_1, doc_object_1)
        self.assertFalse(doc_object_1 < doc_object_1)

        self.assertNotEqual(doc_object_1, doc_object_2)
        self.assertEqual(doc_object_1, doc_object_1)

        self.assertGreater(doc_object_2, doc_object_1)
        self.assertGreaterEqual(doc_object_1, doc_object_1)
        self.assertFalse(doc_object_1 > doc_object_1)

    def test_doc_object_to_string(self):
        """Tests string formatting of DocObjects"""
        name = "TestObject"
        description = "This is a test object."
        doc_object = DocObject(name=name, description=description, var_name=False)

        expected_string = """={}=
{}
""".format(
            name,
            description
        )
        assert doc_object.to_string(1, BaseFormatter()) == expected_string
