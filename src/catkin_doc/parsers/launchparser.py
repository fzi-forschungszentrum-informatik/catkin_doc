import xml.etree.ElementTree as ET

from catkin_doc.datastructures.parameter import LaunchArgument
from catkin_doc.datastructures.launchfile import LaunchFile

class LaunchParser(object):

    def __init__(self, filename):
        self.filename = filename
        self.launchfile = LaunchFile(self.filename.split('/')[-1])
        self.parser_fcts = [('arg', LaunchArgument, self.launchfile.add_argument)]
        self.attributes = {'arg': ('name', 'default', 'value', 'doc')}
        self.tree = ET.parse(filename)
        self.root = self.tree.getroot()
        self.parse()

    def parse(self):
        for tag, type, add in self.parser_fcts:
            for item in self.root.findall(tag):
                name = item.get('name')
                default = item.get('default', default ='')
                comment = item.get('doc', default = '')
                add_item = type(name = name, description = comment, default_value = default)
                if comment == '':
                    add_item.filename = self.filename.split('/')[-1]
                    add_item.code = ET.tostring(item)
                    add_item.line_number = -1
                add(add_item)
