"""Module to parse an existing rst documentation"""

import os
import re
import catkin_doc.node

class RstParser(object):
    """Parser for existing rst files generated with the catkin doc module
       to fill node representation for update"""
    def __init__(self, filename):
        node_name = filename.split(".")[0]
        self.node = catkin_doc.node.Node(node_name)
        with open(filename) as filecontent:
            self.lines = filecontent.readlines()
        self.parse()

    def parse(self):
        """
        Parses the lines extracted from file in init method
        """
        linenumber = 0
        while linenumber < len(self.lines)-1:
            if "Parameter" in self.lines[linenumber]:
                linenumber = self.parse_params(linenumber)
            if "Subscribed Topics" in self.lines[linenumber]:
                linenumber = self.parse_subscribed_topics(linenumber)
            linenumber += 1

    def paragraph_finished(self, linenumber):
        if "-----" in self.lines[linenumber]:
            return True
        else:
            return False

    def parse_params(self, linenumber):
        """
        Parse all parameter from the rst file
        """
        # As next line is the underlining of the heading jump two lines ahead
        linenumber += 2

        # Parse params until next heading which we recognize by the undelining
        while not self.paragraph_finished(linenumber) and linenumber < len(self.lines)-1:
            #search for parametername and default value
            match = re.search('(\S+) \(default value: (\S+)\)', self.lines[linenumber])
            if match:
                param_name = str(match.group(1))
                param_value = None
                if str(match.group(2)) != "-":
                    param_value = str(match.group(2))
                linenumber +=1
                #Extract description of parameter
                description = ""
                while (not re.search('(\S+) \(default value: (\S+)\)', self.lines[linenumber])
                       and not self.paragraph_finished(linenumber) and linenumber < len(self.lines)-1):
                    description += self.lines[linenumber].strip('\n').strip(' ') + " "
                    linenumber +=1

                self.node.add_parameter(param_name, param_value, description)
            else:
                linenumber +=1
        return linenumber-1

    def parse_subscribed_topics(self,linenumber):
        """
        Parse all subscriber from the rst file
        """
        # As next line is the underlining of the heading jump two lines ahead
        linenumber += 2

        # Parse params until next heading which we recognize by the undelining
        while not self.paragraph_finished(linenumber) and linenumber < len(self.lines)-1:
            #search for name of subscribed topic and message type
            match = re.search('(\S+) \((\S+)\)', self.lines[linenumber])
            if match:
                topic = str(match.group(1))
                msg_type = str(match.group(2))
                linenumber +=1
                #Extract description of parameter
                description = ""
                while not re.search('(\S+) \((\S+)\)', lines[linenumber]) and linenumber < len(self.lines)-1 and not self.paragraph_finished(linenumber):
                    description += self.lines[linenumber].strip('\n').strip(' ')+ " "
                    linenumber +=1
                self.node.add_subscriber(topic, msg_type, description)
            else:
                linenumber +=1

        return linenumber-1

