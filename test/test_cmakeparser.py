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

from catkin_doc.parsers.cmakeparser import CMakeParser


class TestCmakeParsing(unittest.TestCase):
    """Test basic functionality of the cmake parsing module"""

    def test_parsing(self):
        """Test parsing of a cmake file"""
        script_dir = os.path.dirname(__file__)  # <-- absolute dir the script is in
        rel_path = 'test_package'
        pkg_dir = os.path.join(script_dir, rel_path)

        parser = CMakeParser(pkg_dir)

        executables = {'test_package_node': [
            os.path.join(pkg_dir, 'src/test_package_node.cpp'),
            os.path.join(pkg_dir, 'src/listener.cpp'),
            os.path.join(pkg_dir, 'include/test_package/listener.h')
        ]}
        self.maxDiff = None

        self.assertDictEqual(parser.executables, executables)
        self.assertEqual(parser.project_name, 'test_package')


if __name__ == '__main__':
    unittest.main()
