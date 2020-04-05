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
import io
import sys
import tempfile

from catkin_doc.parsers.launchparser import LaunchParser
import catkin_doc.datastructures as ds
from catkin_doc.formatters.base_formatter import BaseFormatter

if sys.version_info[0] == 3:
    StringIO = io.StringIO
else:
    StringIO = io.BytesIO


class TestLaunch(unittest.TestCase):
    """Test basic functionality of the launchfile parser module"""

    def test_parsing(self):
        source_code = r'''<launch>
  <arg name="foo" default="bar" doc="foobar"/>
  <arg name="foo1" default="bar"/>
  <arg name="foo2" doc="foobar"/>
  <arg name="foo3"/>
</launch>'''

        source_file = tempfile.NamedTemporaryFile()
        source_file.write(source_code.encode(encoding="utf-8", errors="strict"))
        source_file.seek(0)
        parser = LaunchParser(source_file.name, "/package/root")

        launchfile = parser.launchfile

        self.assertEqual(len(launchfile.children[ds.KEYS["launch_argument"]]), 4)

        self.assertEqual(launchfile.children[ds.KEYS["launch_argument"]][0].name, "foo")
        self.assertEqual(launchfile.children[ds.KEYS["launch_argument"]][0].default_value, "bar")
        self.assertEqual(launchfile.children[ds.KEYS["launch_argument"]][0].description, "foobar")

        self.assertEqual(launchfile.children[ds.KEYS["launch_argument"]][1].name, "foo1")
        self.assertEqual(launchfile.children[ds.KEYS["launch_argument"]][1].default_value, "bar")
        self.assertEqual(launchfile.children[ds.KEYS["launch_argument"]][1].description, None)

        self.assertEqual(launchfile.children[ds.KEYS["launch_argument"]][2].name, "foo2")
        self.assertEqual(launchfile.children[ds.KEYS["launch_argument"]][2].default_value, None)
        self.assertEqual(launchfile.children[ds.KEYS["launch_argument"]][2].description, "foobar")

        self.assertEqual(launchfile.children[ds.KEYS["launch_argument"]][3].name, "foo3")
        self.assertEqual(launchfile.children[ds.KEYS["launch_argument"]][3].default_value, None)
        self.assertEqual(launchfile.children[ds.KEYS["launch_argument"]][3].description, None)

    def test_formatting(self):
        """Test formatting of a launchfile object"""

        doc_object = ds.launchfile.LaunchFile(name="test.launch", description=None)
        formatter = BaseFormatter()

        expected_string = """{}{}

""".format(formatter.heading(1, doc_object.name), doc_object.EMPTY_DESCRIPTION)
        self.assertEqual(doc_object.to_string(1, formatter), expected_string)

        description = "my description"
        doc_object.description = description
        expected_string = """{}{}

""".format(formatter.heading(1, doc_object.name), description)
        self.assertEqual(doc_object.to_string(1, formatter), expected_string)

        description = None
        doc_object.description = description
        arg = ds.parameter.LaunchArgument(
            name="arg1", description="my argumment", default_value=True)
        doc_object.add_argument(arg)
        expected_string = """{}{}{}{}
""".format(formatter.heading(1, doc_object.name),
           "",
           formatter.heading(2, ds.KEYS["launch_argument"]),
           arg.to_string(3, formatter))
        assert expected_string == doc_object.to_string(1, formatter)

        description = "my description"
        doc_object.description = description
        expected_string = """{}{}

{}{}
""".format(formatter.heading(1, doc_object.name),
           description,
           formatter.heading(2, ds.KEYS["launch_argument"]),
           arg.to_string(3, formatter))
        assert expected_string == doc_object.to_string(1, formatter)
