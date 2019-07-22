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
import xml.etree.ElementTree as ET

from catkin_doc.datastructures.parameter import LaunchArgument
from catkin_doc.datastructures.launchfile import LaunchFile


class LaunchParser(object):
    """Parser to parse launch files and fill representation"""

    def __init__(self, filename, package_root):
        self.package_root = package_root
        self.filename = filename
        self.launchfile = LaunchFile(self.filename.split('/')[-1])
        self.parser_fcts = [('arg', LaunchArgument, self.launchfile.add_argument)]
        self.tree = ET.parse(filename)
        self.root = self.tree.getroot()
        self.parse()

    def parse(self):
        """Function to parse xml and fill launchfile representation"""
        for tag, node_t, add in self.parser_fcts:
            for item in self.root.findall(tag):
                name = item.get('name')
                default = item.get('default', default='')
                comment = item.get('doc', default='')
                add_item = node_t(name=name, description=comment, default_value=default)
                if comment == '':
                    add_item.filename = os.path.relpath(self.filename, self.package_root)
                    # add_item.code = ET.tostring(item)
                    add_item.line_number = -1
                add(add_item)
