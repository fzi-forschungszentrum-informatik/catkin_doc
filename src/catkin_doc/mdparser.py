"""Module to parse an existing mark down documentation"""

import os
import re
import catkin_doc.node

class MdParser(object):
    """Parser for existing markdown files generated with the catkin doc module
       to fill node representation for update"""
    def __init__(self, filename):

        #regex for parsing according things
        self.re_param = '\*\*(\S+)\*\* \(default: (\S+)\)'
        self.re_subscriber = '\*\*(\S+)\*\* \(\[?([^]]*)\]?[^]^\s]*\)'
        self.re_publisher = '\*\*(\S+)\*\* \(\[?([^]]*)\]?[^]^\s]*\)'

        if ".md" in filename:
            node_name = filename.split(".")[0]
            self.node = catkin_doc.node.Node(node_name)
            with open(filename) as filecontent:
                self.lines = filecontent.readlines()
            self.parse()
        else:
            print("This is not a markdown file.")
            self.node = None

    def parse(self):
        linenumber = 0

        while linenumber < len(self.lines):
            if "## Parameters" in self.lines[linenumber]:
                linenumber = self.parse_params(linenumber + 1)
            elif "## Subscribed Topics" in self.lines[linenumber]:
                linenumber = self.parse_subscriber(linenumber + 1)
            elif "## Advertised Topics" in self.lines[linenumber]:
                linenumber = self.parse_publisher(linenumber + 1)
            else:
                linenumber += 1

    def paragraph_finished(self, linenumber):
        """
        Helperfunction
        returns true if current aragraph is finished by detecting end of file or new heading
        false otherwise

        """
        if linenumber >= len(self.lines) or "##" in self.lines[linenumber]:
            return True
        else:
            return False

    def extract_description(self, linenumber, pattern):
        """
        Helperfunction for extracting the description
        """
        description = ""
        while (not re.search(pattern, self.lines[linenumber]) and linenumber < len(self.lines)
               and not self.paragraph_finished(linenumber+1)):
            if not self.lines[linenumber].strip(" ").strip("\n") == "":
                description += self.lines[linenumber]
            linenumber +=1

        return linenumber, description

    def parse_params(self, linenumber):
        """
        Function to parse parameter from markdown file and add them to node
        """
        while not self.paragraph_finished(linenumber):
           #extract parameter name and default value
           match = re.search(self.re_param, self.lines[linenumber])
           if match:
               name = str(match.group(1))
               value = None
               if str(match.group(2)) != "-":
                   value = str(match.group(2))
               linenumber +=1
               linenumber, description = self.extract_description(linenumber, self.re_param)
               self.node.add_parameter(name, value,description)
           else:
               linenumber += 1
        return linenumber

    def parse_subscriber(self, linenumber):
        """
        Function to parse subscriber from markdown file and add them to node
        """
        while not self.paragraph_finished(linenumber):
           #extract subscriber topic and msg type
           match = re.search(self.re_subscriber, self.lines[linenumber])
           if match:
               topic = str(match.group(1))
               type = str(match.group(2))
               linenumber +=1
               linenumber, description = self.extract_description(linenumber, self.re_re_subscriber)
               self.node.add_subscriber(topic, type, description)
           else:
               linenumber += 1
        return linenumber

    def parse_publisher(self, linenumber):
        """
        Function to parse publisher from markdown file and add them to node
        """
        while not self.paragraph_finished(linenumber):
           #extract publisher topic and msg type
           match = re.search(self.re_publisher, self.lines[linenumber])
           if match:
               topic = str(match.group(1))
               type = str(match.group(2))
               linenumber +=1
               linenumber, description = self.extract_description(linenumber, self.re_publisher)
               self.node.add_publisher(topic, type, description)
           else:
               linenumber += 1
        return linenumber




