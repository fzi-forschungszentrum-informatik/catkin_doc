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

if __name__ == '__main__':
    unittest.main()
