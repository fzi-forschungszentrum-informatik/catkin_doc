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
