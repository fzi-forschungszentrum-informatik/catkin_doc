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
import tempfile
import unittest

import catkin_doc


class TestFullPackage(unittest.TestCase):
    """Test the full documentation generation"""

    def test_generation(self):
        """Test generating documentation generation"""

        script_dir = os.path.dirname(__file__)  # <-- absolute dir the script is in
        rel_path = 'test_package'
        pkg_path = os.path.join(script_dir, rel_path)
        ref_doc = os.path.join(pkg_path, 'README.md')
        pkg_handler = catkin_doc.pkghandler.PkgHandler(pkg_path, 'ignore_existing_doc')

        formatter = catkin_doc.formatters.markdown_formatter.MarkdownFormatter()
        out_string = pkg_handler.doc.to_string(1, formatter)

        with open(ref_doc) as ref_file:
            self.assertEqual(out_string, ref_file.read())

    def test_round_trip(self):
        """Test generating documentation generation and merging it with the existing one"""

        script_dir = os.path.dirname(__file__)  # <-- absolute dir the script is in
        rel_path = 'test_package'
        pkg_path = os.path.join(script_dir, rel_path)
        ref_doc = os.path.join(pkg_path, 'README.md')
        pkg_handler = catkin_doc.pkghandler.PkgHandler(pkg_path, 'README.md')

        formatter = catkin_doc.formatters.markdown_formatter.MarkdownFormatter()
        out_string = pkg_handler.doc.to_string(1, formatter)

        self.maxDiff = None
        with open(ref_doc) as ref_file:
            self.assertEqual(out_string, ref_file.read())

if __name__ == '__main__':
    unittest.main()
