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


import unittest
import ast
import tokenize
from io import StringIO
# import os.path

# import catkin_doc.datastructures as ds
import catkin_doc.parsers.pythonparser as pythonparser
# from catkin_doc.datastructures.parameter import Parameter
# from catkin_doc.datastructures.service import Service, ServiceClient
# from catkin_doc.datastructures.action import Action, ActionClient
# from catkin_doc.datastructures.topic import Subscriber, Publisher


class TestPython(unittest.TestCase):
    """Test basic functionality of the python doc module"""

    def test_parsing(self):
        source_code = r'''from std_msgs.msg import String
from std_srvs.srv import Trigger as Trigg
self.pub = rospy.Publisher("pub_topic", String, queue_size=1)
self.pub = rospy.Subscriber("sub_topic", String)
self.service = rospy.Service('service_name', Trigg, trigger_cb)
self.param = rospy.get_param('param_name', "default_value")
param_name = "my_param"
self.param2 = rospy.get_param(param_name, 1.0)
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

        tree = ast.parse(source_code)
        tokens = list()
        for five_tuple in tokenize.generate_tokens(StringIO(source_code).readline):
            tokens.append(five_tuple)

        analyzer = pythonparser.Analyzer(tokens)
        analyzer.visit(tree)

        expected = {'Publisher': [{'comment': '',
                                   'lineno': 3,
                                   'topic': 'pub_topic',
                                   'type': 'std_msgs/String'}],
                    'Subscriber': [{'comment': '',
                                    'lineno': 4,
                                    'topic': 'sub_topic',
                                    'type': 'std_msgs/String'}],
                    'Service': [{'comment': '',
                                 'lineno': 5,
                                 'topic': 'service_name',
                                 'type': 'std_srvs/Trigger'}],
                    'get_param': [{'comment': '',
                                   'lineno': 6,
                                   'name': 'param_name',
                                   'default': 'default_value'},
                                  {'comment': '',
                                   'lineno': 8,
                                   'name': "id: param_name",
                                   'default': 1.0},
                                  {'comment': '',
                                   'lineno': 9,
                                   'name': "param_name3",
                                   'default': ''},
                                  {'comment': '',
                                   'lineno': 10,
                                   'name': "param_name4",
                                   'default': 'id: param_name'},
                                  {'comment': '',
                                   'lineno': 11,
                                   'name': "param_list",
                                   'default': [1, 2, 3]},
                                  {'comment': '',
                                   'lineno': 12,
                                   'name': "~param_dict",
                                   'default': {'a': 1, 'b': 2, 'c': 3}},
                                  {'comment': '',
                                   'lineno': 13,
                                   'name': "~param_tuple",
                                   'default': "UNKNOWN_TYPE"}],
                    'SimpleActionServer': [{'comment': '',
                                            'lineno': 16,
                                            'topic': 'id: action_name',
                                            'type': 'actionlib_tutorials/FibonacciAction'}],
                    'SimpleActionClient': [{'comment': '',
                                            'lineno': 17,
                                            'topic': 'fibonacci',
                                            'type': 'actionlib_tutorials/FibonacciAction'}],
                    'ServiceProxy': [{'comment': 'AddTwoInts is unknown due to wildcard export',
                                      'lineno': 20,
                                      'topic': 'add_two_ints',
                                      'type': 'id: AddTwoInts'}],
                    }

        subset = {k:v for k, v in analyzer.stats.items() if k in expected}
        self.assertDictEqual(subset, expected)

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
                                    'lineno': 6,
                                    'topic': 'sub_topic',
                                    'type': 'std_msgs/String'}],
                    }

        subset = {k:v for k, v in analyzer.stats.items() if k in expected}
        self.assertDictEqual(subset, expected)

if __name__ == '__main__':
    unittest.main()
