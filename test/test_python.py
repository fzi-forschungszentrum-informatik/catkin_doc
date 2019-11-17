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


import ast
import io
import os
import sys
import tempfile
import tokenize
import unittest

import os.path

import catkin_doc.datastructures as ds
import catkin_doc.parsers.pythonparser as pythonparser
# from catkin_doc.datastructures.parameter import Parameter
# from catkin_doc.datastructures.service import Service, ServiceClient
# from catkin_doc.datastructures.action import Action, ActionClient
# from catkin_doc.datastructures.topic import Subscriber, Publisher

if sys.version_info[0] == 3:
    StringIO = io.StringIO
else:
    StringIO = io.BytesIO


class TestPython(unittest.TestCase):
    """Test basic functionality of the python doc module"""
    source_code = r'''from std_msgs.msg import String
from std_srvs.srv import Trigger as Trigg
self.pub = rospy.Publisher("pub_topic", String, queue_size=1)
self.pub = rospy.Subscriber("sub_topic", String)
self.service = rospy.Service('service_name', Trigg, trigger_cb)
self.param = rospy.get_param('param_name', "default_value")
param_name_var = "my_param"
self.param2 = rospy.get_param(param_name_var, 1.0)
self.param3 = rospy.get_param("param_name3")
self.param4 = rospy.get_param("param_name4", param_name)
self.param_list = rospy.get_param("param_list", [1, 2, 3])
self.param_dict = rospy.get_param("~param_dict", {'a': 1, 'b': 2, 'c': 3})
self.param_tuple = rospy.get_param("~param_tuple", (1, 2)) # illegal
import actionlib_tutorials
action_name = "example_action"
self.action = actionlib.SimpleActionServer(action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
client = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)
from beginner_tutorials.srv import *
# AddTwoInts is unknown due to wildcard export
add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
'''

    def test_parsing(self):
        tree = ast.parse(self.source_code)
        tokens = list()
        for five_tuple in tokenize.generate_tokens(StringIO(self.source_code).readline):
            tokens.append(five_tuple)

        analyzer = pythonparser.Analyzer(tokens)
        analyzer.visit(tree)
        self.maxDiff = None

        expected = {'Publisher': [{'comment': '',
                                   'is_symbol': False,
                                   'lineno': 3,
                                   'topic': 'pub_topic',
                                   'type': 'std_msgs/String'}],
                    'Subscriber': [{'comment': '',
                                    'is_symbol': False,
                                    'lineno': 4,
                                    'topic': 'sub_topic',
                                    'type': 'std_msgs/String'}],
                    'Service': [{'comment': '',
                                 'is_symbol': False,
                                 'lineno': 5,
                                 'topic': 'service_name',
                                 'type': 'std_srvs/Trigger'}],
                    'get_param': [{'comment': '',
                                   'is_symbol': False,
                                   'lineno': 6,
                                   'name': 'param_name',
                                   'default': 'default_value'},
                                  {'comment': '',
                                   'is_symbol': True,
                                   'lineno': 8,
                                   'name': "param_name_var",
                                   'default': 1.0},
                                  {'comment': '',
                                   'is_symbol': False,
                                   'lineno': 9,
                                   'name': "param_name3",
                                   'default': None},
                                  {'comment': '',
                                   'is_symbol': False,
                                   'lineno': 10,
                                   'name': "param_name4",
                                   'default': 'id: param_name'},
                                  {'comment': '',
                                   'is_symbol': False,
                                   'lineno': 11,
                                   'name': "param_list",
                                   'default': [1, 2, 3]},
                                  {'comment': '',
                                   'is_symbol': False,
                                   'lineno': 12,
                                   'name': "~param_dict",
                                   'default': {'a': 1, 'b': 2, 'c': 3}},
                                  {'comment': '',
                                   'is_symbol': False,
                                   'lineno': 13,
                                   'name': "~param_tuple",
                                   'default': "UNKNOWN_TYPE"}],
                    'SimpleActionServer': [{'comment': '',
                                            'is_symbol': True,
                                            'lineno': 16,
                                            'topic': 'action_name',
                                            'type': 'actionlib_tutorials/FibonacciAction'}],
                    'SimpleActionClient': [{'comment': '',
                                            'is_symbol': False,
                                            'lineno': 17,
                                            'topic': 'fibonacci',
                                            'type': 'actionlib_tutorials/FibonacciAction'}],
                    'ServiceProxy': [{'comment': 'AddTwoInts is unknown due to wildcard export',
                                      'is_symbol': False,
                                      'lineno': 20,
                                      'topic': 'add_two_ints',
                                      'type': 'AddTwoInts'}],
                    }

        subset = {k: v for k, v in analyzer.stats.items() if k in expected}
        self.assertDictEqual(subset, expected)

    def test_create_objects(self):
        """Test creating objects from parsed code"""
        source_file = tempfile.NamedTemporaryFile()
        source_file.write(self.source_code.encode(encoding="utf-8", errors="strict"))
        source_file.seek(0)
        parser = pythonparser.PythonParser(source_file.name)
        data = source_file.readlines()
        for number, line in enumerate(data):
            print("{}: {}".format(number + 1, line))

        node = parser.node

        self.assertEqual(node.name, os.path.basename(source_file.name))
        self.assertEqual(len(node.children), 7)
        self.assertEqual(node.children[ds.KEYS["publisher"]][0].name, "pub_topic")
        self.assertEqual(node.children[ds.KEYS["publisher"]][0].namespace, None)
        self.assertEqual(node.children[ds.KEYS["publisher"]][0].line_number, 3)
        self.assertEqual(node.children[ds.KEYS["publisher"]][0].datatype, "std_msgs/String")
        self.assertEqual(node.children[ds.KEYS["publisher"]][0].var_name, False)
        self.assertEqual(node.children[ds.KEYS["publisher"]][0].code,
                         data[2].decode("utf-8").lstrip(' '))

        self.assertEqual(node.children[ds.KEYS["subscriber"]][0].name, "sub_topic")
        self.assertEqual(node.children[ds.KEYS["subscriber"]][0].namespace, None)
        self.assertEqual(node.children[ds.KEYS["subscriber"]][0].line_number, 4)
        self.assertEqual(node.children[ds.KEYS["subscriber"]][0].datatype, "std_msgs/String")
        self.assertEqual(node.children[ds.KEYS["subscriber"]][0].var_name, False)
        self.assertEqual(node.children[ds.KEYS["subscriber"]][0].code,
                         data[3].decode("utf-8").lstrip(' '))

        self.assertEqual(node.children[ds.KEYS["service"]][0].name, "service_name")
        self.assertEqual(node.children[ds.KEYS["service"]][0].namespace, None)
        self.assertEqual(node.children[ds.KEYS["service"]][0].line_number, 5)
        self.assertEqual(node.children[ds.KEYS["service"]][0].datatype, "std_srvs/Trigger")
        self.assertEqual(node.children[ds.KEYS["service"]][0].var_name, False)
        self.assertEqual(node.children[ds.KEYS["service"]][0].code,
                         data[4].decode("utf-8").lstrip(' '))

        self.assertEqual(node.children[ds.KEYS["parameter"]][0].name, "param_name")
        self.assertEqual(node.children[ds.KEYS["parameter"]][0].namespace, None)
        self.assertEqual(node.children[ds.KEYS["parameter"]][0].line_number, 6)
        self.assertEqual(node.children[ds.KEYS["parameter"]][0].default_value, "default_value")
        self.assertEqual(node.children[ds.KEYS["parameter"]][0].var_name, False)
        self.assertEqual(node.children[ds.KEYS["parameter"]][0].code,
                         data[5].decode("utf-8").lstrip(' '))

        self.assertEqual(node.children[ds.KEYS["parameter"]][1].name, "param_name_var")
        self.assertEqual(node.children[ds.KEYS["parameter"]][1].namespace, None)
        self.assertEqual(node.children[ds.KEYS["parameter"]][1].line_number, 8)
        self.assertEqual(node.children[ds.KEYS["parameter"]][1].default_value, 1.0)
        self.assertEqual(node.children[ds.KEYS["parameter"]][1].var_name, True)
        self.assertEqual(node.children[ds.KEYS["parameter"]][1].code,
                         data[7].decode("utf-8").lstrip(' '))

        self.assertEqual(node.children[ds.KEYS["parameter"]][2].name, "param_name3")
        self.assertEqual(node.children[ds.KEYS["parameter"]][2].namespace, None)
        self.assertEqual(node.children[ds.KEYS["parameter"]][2].line_number, 9)
        self.assertEqual(node.children[ds.KEYS["parameter"]][2].default_value, None)
        self.assertEqual(node.children[ds.KEYS["parameter"]][2].var_name, False)
        self.assertEqual(node.children[ds.KEYS["parameter"]][2].code,
                         data[8].decode("utf-8").lstrip(' '))

        self.assertEqual(node.children[ds.KEYS["parameter"]][3].name, "param_name4")
        self.assertEqual(node.children[ds.KEYS["parameter"]][3].namespace, None)
        self.assertEqual(node.children[ds.KEYS["parameter"]][3].line_number, 10)
        self.assertEqual(node.children[ds.KEYS["parameter"]][3].default_value, "id: param_name")
        self.assertEqual(node.children[ds.KEYS["parameter"]][3].var_name, False)
        self.assertEqual(node.children[ds.KEYS["parameter"]][3].code,
                         data[9].decode("utf-8").lstrip(' '))

        self.assertEqual(node.children[ds.KEYS["parameter"]][4].name, "param_list")
        self.assertEqual(node.children[ds.KEYS["parameter"]][4].namespace, None)
        self.assertEqual(node.children[ds.KEYS["parameter"]][4].line_number, 11)
        self.assertEqual(node.children[ds.KEYS["parameter"]][4].default_value, [1, 2, 3])
        self.assertEqual(node.children[ds.KEYS["parameter"]][4].var_name, False)
        self.assertEqual(node.children[ds.KEYS["parameter"]][4].code,
                         data[10].decode("utf-8").lstrip(' '))

        self.assertEqual(node.children[ds.KEYS["parameter"]][5].name, "param_dict")
        self.assertEqual(node.children[ds.KEYS["parameter"]][5].namespace, "~")
        self.assertEqual(node.children[ds.KEYS["parameter"]][5].line_number, 12)
        self.assertEqual(node.children[ds.KEYS["parameter"]]
                         [5].default_value, {'a': 1, 'b': 2, 'c': 3})
        self.assertEqual(node.children[ds.KEYS["parameter"]][5].var_name, False)
        self.assertEqual(node.children[ds.KEYS["parameter"]][5].code,
                         data[11].decode("utf-8").lstrip(' '))

        self.assertEqual(node.children[ds.KEYS["parameter"]][6].name, "param_tuple")
        self.assertEqual(node.children[ds.KEYS["parameter"]][6].namespace, "~")
        self.assertEqual(node.children[ds.KEYS["parameter"]][6].line_number, 13)
        self.assertEqual(node.children[ds.KEYS["parameter"]][6].default_value, "UNKNOWN_TYPE")
        self.assertEqual(node.children[ds.KEYS["parameter"]][6].var_name, False)
        self.assertEqual(node.children[ds.KEYS["parameter"]][6].code,
                         data[12].decode("utf-8").lstrip(' '))

        self.assertEqual(node.children[ds.KEYS["action"]][0].name, "action_name")
        self.assertEqual(node.children[ds.KEYS["action"]][0].namespace, None)
        self.assertEqual(node.children[ds.KEYS["action"]][0].line_number, 16)
        self.assertEqual(node.children[ds.KEYS["action"]][0].datatype,
                         "actionlib_tutorials/FibonacciAction")
        self.assertEqual(node.children[ds.KEYS["action"]][0].var_name, True)
        self.assertEqual(node.children[ds.KEYS["action"]][0].code,
                         data[15].decode("utf-8").lstrip(' '))

        self.assertEqual(node.children[ds.KEYS["action_client"]][0].name, "fibonacci")
        self.assertEqual(node.children[ds.KEYS["action_client"]][0].namespace, None)
        self.assertEqual(node.children[ds.KEYS["action_client"]][0].line_number, 17)
        self.assertEqual(node.children[ds.KEYS["action_client"]][0].datatype,
                         "actionlib_tutorials/FibonacciAction")
        self.assertEqual(node.children[ds.KEYS["action_client"]][0].var_name, False)
        self.assertEqual(node.children[ds.KEYS["action_client"]][0].code,
                         data[16].decode("utf-8").lstrip(' '))

        self.assertEqual(node.children[ds.KEYS["service_client"]][0].name, "add_two_ints")
        self.assertEqual(node.children[ds.KEYS["service_client"]][0].namespace, None)
        self.assertEqual(node.children[ds.KEYS["service_client"]][0].description,
                         "AddTwoInts is unknown due to wildcard export")
        self.assertEqual(node.children[ds.KEYS["service_client"]][0].line_number, 20)
        self.assertEqual(node.children[ds.KEYS["service_client"]][0].datatype,
                         "AddTwoInts")
        self.assertEqual(node.children[ds.KEYS["service_client"]][0].var_name, False)
        self.assertEqual(node.children[ds.KEYS["service_client"]][0].code,
                         data[19].decode("utf-8").lstrip(' '))

    def test_comment_search(self):
        source_code = r'''from std_msgs.msg import String
# no comment

# Comment for subscriber1
# Comment for subscriber2
self.pub = rospy.Subscriber("sub_topic", String)
'''

        tree = ast.parse(source_code)
        tokens = list()
        for five_tuple in tokenize.generate_tokens(StringIO(source_code).readline):
            tokens.append(five_tuple)

        analyzer = pythonparser.Analyzer(tokens)
        analyzer.visit(tree)

        expected = {'Subscriber': [{'comment': 'Comment for subscriber1 Comment for subscriber2',
                                    'is_symbol': False,
                                    'lineno': 6,
                                    'topic': 'sub_topic',
                                    'type': 'std_msgs/String'}],
                    }

        subset = {k: v for k, v in analyzer.stats.items() if k in expected}
        self.assertDictEqual(subset, expected)

    def test_python_node(self):
        """Test parsing of a python node file"""
        script_dir = os.path.dirname(__file__)  # <-- absolute dir the script is in
        rel_path = 'test_package/scripts/python_node'
        abs_file_path = os.path.join(script_dir, rel_path)
        with open(abs_file_path, 'r') as source_file:
            source_code = source_file.read()
        tree = ast.parse(source_code)
        tokens = list()
        for five_tuple in tokenize.generate_tokens(StringIO(source_code).readline):
            tokens.append(five_tuple)

        analyzer = pythonparser.Analyzer(tokens)
        analyzer.visit(tree)

        self.maxDiff = None

        expected = {'Subscriber': [{'comment': '',
                                    'is_symbol': False,
                                    'lineno': 14,
                                    'topic': 'topic1',
                                    'type': 'std_msgs/Bool'},
                                   {'comment': 'This is a comment for sub2',
                                    'is_symbol': False,
                                    'lineno': 17,
                                    'topic': 'topic2',
                                    'type': 'std_msgs/Bool'},
                                   {'comment': '',
                                    'is_symbol': False,
                                    'lineno': 19,
                                    'topic': 'topic3',
                                    'type': 'std_msgs/Bool'}],
                    }

        subset = {k: v for k, v in analyzer.stats.items() if k in expected}
        self.assertDictEqual(subset, expected)


if __name__ == '__main__':
    unittest.main()
