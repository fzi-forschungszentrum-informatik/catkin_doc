"""Module to parse an existing rst documentation"""

import os
import re
import catkin_doc.node

class RstParser(object):
    """Parser for existing rst files generated with the catkin doc module
       to fill node representation for update"""
    def __init__(self, filename):
        if ".rst" in filename:
            node_name = filename.split(".")[0]
            self.node = catkin_doc.node.Node(node_name)
            with open(filename) as filecontent:
                self.lines = filecontent.readlines()
            self.parse()
        else:
            print("This is not an rst file.")
            self.node = None

    def parse(self):
        """
        Parses the lines extracted from file in init method
        """
        linenumber = 0
        while linenumber < len(self.lines)-1:
            if "Parameter" in self.lines[linenumber]:
                linenumber = self.parse_params(linenumber)
            elif "Subscribed topics" in self.lines[linenumber]:
                linenumber = self.parse_subscribed_topics(linenumber)
            elif "Advertised topics" in self.lines[linenumber]:
                linenumber = self.parse_advertised_topics(linenumber)
            elif "Service clients" in self.lines[linenumber]:
                linenumber = self.parse_service_clients(linenumber)
            elif "Advertised services" in self.lines[linenumber]:
                linenumber = self.parse_services(linenumber)
            elif "Action clients" in self.lines[linenumber]:
                linenumber = self.parse_action_clients(linenumber)
            elif "Action server" in self.lines[linenumber]:
                linenumber = self.parse_action_server(linenumber)
            else:
                linenumber += 1

    def paragraph_finished(self, linenumber):
        """
        Helperfunction to recognize underlinings which indicate new paragraph
        """
        if "-----" in self.lines[linenumber]:
            return True
        else:
            return False

    def extract_description(self, linenumber, pattern):
        """
        Helperfunction for extracting the description
        """
        description = ""
        while (not re.search(pattern, self.lines[linenumber])
               and not self.paragraph_finished(linenumber) and linenumber < len(self.lines)-1):
            description += self.lines[linenumber].strip('\n').strip(' ') + " "
            linenumber +=1

        return linenumber, description

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
                linenumber, description = self.extract_description(linenumber, '(\S+) \(default value: (\S+)\)')
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
        # Parse subscribed topics until next heading which we recognize by the underlining
        while not self.paragraph_finished(linenumber) and linenumber < len(self.lines)-1:
            #search for name of subscribed topic and message type
            match = re.search('(\S+) \((\S+)\s?\S*\)', self.lines[linenumber])
            if match:
                topic = str(match.group(1))
                msg_type = str(match.group(2))
                linenumber +=1
                #Extract description of parameter
                linenumber, description = self.extract_description(linenumber, '(\S+) \((\S+\s?\S*)\)')
                self.node.add_subscriber(topic, msg_type, description)
            else:
                linenumber +=1
        return linenumber-1

    def parse_advertised_topics(self,linenumber):
        """
        Parse all publisher from the rst file
        """
        # As next line is the underlining of the heading jump two lines ahead
        linenumber += 2
        # Parse advertised topics until next heading which we recognize by the underlining
        while not self.paragraph_finished(linenumber) and linenumber < len(self.lines)-1:
            #search for name of advertised topic and message type
            match = re.search('(\S+) \((\S+)\s?\S*\)', self.lines[linenumber])
            if match:
                topic = str(match.group(1))
                msg_type = str(match.group(2))
                linenumber +=1
                #Extract description of parameter
                linenumber, description = self.extract_description(linenumber, '(\S+) \((\S+\s?\S*)\)')
                self.node.add_publisher(topic, msg_type, description)
            else:
                linenumber +=1
        return linenumber-1

    def parse_service_clients(self, linenumber):
        """
        Parse all service clients from the rst file
        """
        # As next line is the underlining of the heading jump two lines ahead
        linenumber += 2
        # Parse service clients until next heading which we recognize by the underlining
        while not self.paragraph_finished(linenumber) and linenumber < len(self.lines)-1:
            #search for topic of the service client and message type
            match = re.search('(\S+) \((\S+)\s?\S*\)', self.lines[linenumber])
            if match:
                topic = str(match.group(1))
                msg_type = str(match.group(2))
                linenumber +=1
                #Extract description of parameter
                linenumber, description = self.extract_description(linenumber, '(\S+) \((\S+\s?\S*)\)')
                self.node.add_service_client(topic, msg_type, description)
            else:
                linenumber +=1
        return linenumber-1

    def parse_services(self, linenumber):
        """
        Parse all services from the rst file
        """
        # As next line is the underlining of the heading jump two lines ahead
        linenumber += 2
        # Parse services until next heading which we recognize by the underlining
        while not self.paragraph_finished(linenumber) and linenumber < len(self.lines)-1:
            #search for the service name and message type
            match = re.search('(\S+) \((\S+)\s?\S*\)', self.lines[linenumber])
            if match:
                topic = str(match.group(1))
                msg_type = str(match.group(2))
                linenumber +=1
                #Extract description of parameter
                linenumber, description = self.extract_description(linenumber, '(\S+) \((\S+\s?\S*)\)')
                self.node.add_service(topic, msg_type, description)
            else:
                linenumber +=1
        return linenumber-1

    def parse_action_clients(self, linenumber):
        """
        Parse all action clients from the rst file
        """
        # As next line is the underlining of the heading jump two lines ahead
        linenumber += 2
        # Parse action clients until next heading which we recognize by the underlining
        while not self.paragraph_finished(linenumber) and linenumber < len(self.lines)-1:
            #search for the action clients name and message type
            match = re.search('(\S+) \((\S+)\s?\S*\)', self.lines[linenumber])
            if match:
                topic = str(match.group(1))
                msg_type = str(match.group(2))
                linenumber +=1
                #Extract description of parameter
                linenumber, description = self.extract_description(linenumber, '(\S+) \((\S+\s?\S*)\)')
                self.node.add_action_client(topic, msg_type, description)
            else:
                linenumber +=1
        return linenumber-1

    def parse_action_server(self, linenumber):
        """
        Parse all action server from the rst file
        """
        # As next line is the underlining of the heading jump two lines ahead
        linenumber += 2
        # Parse action server until next heading which we recognize by the underlining
        while not self.paragraph_finished(linenumber) and linenumber < len(self.lines)-1:
            #search for the action clients name and message type
            match = re.search('(\S+\s?\S*) \((\S+)\s?\S*\)', self.lines[linenumber])
            if match:
                topic = str(match.group(1))
                msg_type = str(match.group(2))
                linenumber +=1
                #Extract description of parameter
                linenumber, description = self.extract_description(linenumber, '(\S+\s?\S*) \((\S+\s?\S*)\)')
                self.node.add_action(topic, msg_type, description)
            else:
                linenumber +=1
        return linenumber-1

