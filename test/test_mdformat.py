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

import catkin_doc.datastructures as ds
import catkin_doc.formatters.markdown_formatter as mdformatter


class TestFormatting(unittest.TestCase):
    """Tests formatting using the Markdown formatter."""

    def test_parameters(self):
        """Test formatting of paramter objects"""
        my_param = ds.parameter.Parameter("param_name",
                                          description="This is a parameter",
                                          default_value=1.5,
                                          var_name=False)

        formatter = mdformatter.MarkdownFormatter()
        formatted_string = my_param.to_string(1, formatter)
        expected_string = r'''"**param_name**" (default: 1.5)

This is a parameter
'''

        self.assertEqual(formatted_string, expected_string)

        my_param = ds.parameter.Parameter("param_name_2",
                                          description="This is a parameter",
                                          var_name=False)

        formatter = mdformatter.MarkdownFormatter()
        formatted_string = my_param.to_string(1, formatter)
        expected_string = r'''"**param_name_2**" (Required)

This is a parameter
'''

        self.assertEqual(formatted_string, expected_string)

        my_param = ds.parameter.Parameter("param_name_sym",
                                          description="This is a symbol",
                                          var_name=True)

        formatter = mdformatter.MarkdownFormatter()
        formatted_string = my_param.to_string(1, formatter)
        expected_string = r'''"Symbol: **param_name_sym**" (Required)

This is a symbol
'''

        self.assertEqual(formatted_string, expected_string)

        my_param = ds.parameter.Parameter("param_string",
                                          default_value="hello",
                                          description="This is a symbol",
                                          var_name=True)

        formatter = mdformatter.MarkdownFormatter()
        formatted_string = my_param.to_string(1, formatter)
        expected_string = r'''"Symbol: **param_string**" (default: "hello")

This is a symbol
'''
        self.assertEqual(formatted_string, expected_string)

        my_param = ds.parameter.Parameter("param_string",
                                          default_value="hello",
                                          description=None,
                                          var_name=False)
        my_param.code = r"""  param_string = rospy.param("~param_string", default("hello"))"""
        my_param.line_number = 1

        # Test default description
        formatter = mdformatter.MarkdownFormatter()
        formatted_string = my_param.to_string(1, formatter)
        expected_string = r'''"**param_string**" (default: "hello")

Please add description. See  line number: 1



	  param_string = rospy.param("~param_string", default("hello"))
'''

        self.assertEqual(formatted_string, expected_string)

        # Test no code
        my_param = ds.parameter.Parameter("param_string",
                                          default_value="hello",
                                          description=None,
                                          var_name=False)
        formatter = mdformatter.MarkdownFormatter()

        with self.assertRaises(RuntimeError):
            formatted_string = my_param.to_string(1, formatter)

    def test_launch_arguments(self):
        """Test formatting of paramter objects"""
        # Default entry
        my_arg = ds.parameter.LaunchArgument("arg_name",
                                             description="This is a launch argument.",
                                             default_value=1.5,
                                             var_name=False)
        formatter = mdformatter.MarkdownFormatter()
        formatted_string = my_arg.to_string(1, formatter)
        expected_string = r'''"**arg_name**" (default: 1.5)

This is a launch argument.
'''
        self.assertEqual(formatted_string, expected_string)

        # No description
        my_arg = ds.parameter.LaunchArgument("arg_name",
                                             description=None,
                                             default_value=1.5,
                                             var_name=False)
        my_arg.filename = "test_launch.launch"
        my_arg.line_number = 1
        formatter = mdformatter.MarkdownFormatter()
        formatted_string = my_arg.to_string(1, formatter)
        expected_string = r'''"**arg_name**" (default: 1.5)

Please add description. See file "test_launch.launch".
'''
        self.assertEqual(formatted_string, expected_string)

        # Test no line
        my_arg = ds.parameter.LaunchArgument("param_string",
                                             default_value=None,
                                             description=None,
                                             var_name=False)
        formatter = mdformatter.MarkdownFormatter()

        with self.assertRaises(RuntimeError):
            formatted_string = my_arg.to_string(1, formatter)

    def test_subscribers(self):
        """Test formatting of Topic objects"""
        # Default entry
        my_topic = ds.topic.Topic("topic_name",
                                  description="This is a fancy topic",
                                  datatype="std_msgs/Bool",
                                  var_name=False)
        formatter = mdformatter.MarkdownFormatter()
        formatted_string = my_topic.to_string(1, formatter)
        expected_string = r'''"**topic_name**" ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))

This is a fancy topic
'''
        self.assertEqual(formatted_string, expected_string)

        # Custom datatype
        my_topic = ds.topic.Topic("topic_name",
                                  description="This is a fancy topic",
                                  datatype="my_own_msgs/Imaginary",
                                  var_name=False)
        formatter = mdformatter.MarkdownFormatter()
        formatted_string = my_topic.to_string(1, formatter)
        expected_string = r'''"**topic_name**" (my_own_msgs/Imaginary)

This is a fancy topic
'''
        self.assertEqual(formatted_string, expected_string)

        # Symbol name
        my_topic = ds.topic.Topic("topic_name",
                                  description="This is a fancy topic",
                                  datatype="std_msgs/Bool",
                                  var_name=True)
        formatter = mdformatter.MarkdownFormatter()
        formatted_string = my_topic.to_string(1, formatter)
        expected_string = r'''"Symbol: **topic_name**" ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))

This is a fancy topic
'''
        self.assertEqual(formatted_string, expected_string)

        # No description
        my_topic = ds.topic.Topic("topic_name",
                                  description=None,
                                  datatype="std_msgs/Bool",
                                  var_name=True)
        my_topic.line_number = 1
        my_topic.code = r"""  self.pub = rospy.Publisher(topic_name)"""
        formatter = mdformatter.MarkdownFormatter()
        formatted_string = my_topic.to_string(1, formatter)
        expected_string = r'''"Symbol: **topic_name**" ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))

Please add description. See  line number: 1



	  self.pub = rospy.Publisher(topic_name)
'''
        self.assertEqual(formatted_string, expected_string)


if __name__ == '__main__':
    unittest.main()
