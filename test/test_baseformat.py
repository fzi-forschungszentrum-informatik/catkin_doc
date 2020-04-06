#!/usr/bin/env python
# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright (c) 2020, FZI Forschungszentrum Informatik
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
import catkin_doc.formatters.base_formatter as baseformatter

class TestFormatting(unittest.TestCase):
    """Tests formatting using the Base formatter."""

    def test_text_format(self):
        """Tests basic text formatting"""
        formatter = baseformatter.BaseFormatter()
        text = """This is
my text."""

        expected_string = """This is
my text.
"""
        self.assertEqual(formatter.text(text), expected_string)
        self.assertEqual(formatter.text(text, False), text)

    def test_bold(self):
        """Tests bold text formatting"""
        formatter = baseformatter.BaseFormatter()
        text = """This is my text."""

        self.assertEqual(formatter.bold(text), text)

    def test_as_list_item(self):
        """Tests formatting as a list item"""
        formatter = baseformatter.BaseFormatter()
        text = """This is
my text."""

        expected_string = """ * This is
my text."""
        self.assertEqual(formatter.as_list_item(1, text), expected_string)

    def test_link(self):
        """Tests formatting of links"""
        formatter = baseformatter.BaseFormatter()
        text = "my link"
        link = """http://www.test.link"""

        self.assertEqual(formatter.link(link), link)
        self.assertEqual(formatter.link(link, text), text)
        self.assertEqual(formatter.link(link, ""), link)
        self.assertEqual(formatter.link(link, None), link)
