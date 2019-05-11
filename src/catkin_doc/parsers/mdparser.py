"""Module to parse an existing mark down documentation"""

import os
import re

import catkin_doc.datastructures as ds

class DocSection(object):
    """Small helper class representing a hierarchy level inside a doc file"""
    def __init__(self, lines, doc_object_type=ds.DocObject, level=0):
        self.lines = lines
        self.level = level
        self.package_t = doc_object_type
        self.doc_object = None
        self.children = dict()

        self.parse()

    def parse(self):
        sub_lines = None
        current_title = None
        for line in self.lines:
            match = re.search("^#{" + str(self.level+1) + "} ?([^#].*)", line)
            if match:
                print("{}Found current level's title: {}".format(self.level*" ", match.group(1)))
                self.doc_object = self.package_t(match.group(1))
            elif not self.doc_object:
                # If we haven't found a name, continue until we do
                continue

            match = re.search("^#{" + str(self.level+2) + "} ?([^#].*)", line)
            if match:
                print("{}Found child: {}".format(self.level*" ", match.group(1)))
                if sub_lines:
                    # children_type = create_doc_object(match.group(1))
                    self.children[current_title] = DocSection(
                        sub_lines, level=self.level+1)
                current_title = match.group(1)
                sub_lines = [line]
            elif sub_lines:
                # add line to currently creating section
                sub_lines.append(line)
            else:
                continue
        if sub_lines:
            self.children[current_title] = DocSection( sub_lines, level=self.level+1)


class MdParser(object):
    """Parser for existing markdown files generated with the catkin doc module
       to fill node representation for update"""
    def __init__(self, filename, starting_line=0):
        self.starting_line = starting_line
        self.current_level = 0
        self.doc_object = ds.DocObject("")
        self.doc = None

        #regex for parsing according things
        self.re_param = '\*\*(\S+)\*\* \(default: (\S+)\)'
        self.re_subscriber = '\*\*(\S+)\*\* \(\[?([^\]^\)]*)\]?[^]^\s]*\)'
        self.re_publisher = '\*\*(\S+)\*\* \(\[?([^\]^\)]*)\]?[^]^\s]*\)'
        self.re_service_clients = '\*\*(\S+)\*\* \(\[?([^\]^\)]*)\]?[^]^\s]*\)'
        self.re_services = '\*\*(\S+)\*\* \(\[?([^\]^\)]*)\]?[^]^\s]*\)'
        self.re_action_clients = '\*\*(\S+)\*\* \(\[?([^\]^\)]*)\]?[^]^\s]*\)'
        self.re_actions = '\*\*(\S+\s?\S*)\*\* \(\[?([^\]^\)]*)\]?[^]^\s]*\)'

        if ".md" in filename:
            with open(filename) as filecontent:
                lines = filecontent.readlines()
            for linenumber in range(len(lines)):
                match = re.search("^# ?([^#].*)", lines[linenumber])
                if match:
                    package_name = match.group(1)
                    self.doc_object.name = package_name
                    self.doc = DocSection(lines[linenumber:], ds.Package, level=0)
                    break
        else:
            print("This is not a markdown file.")


    def paragraph_finished(self, linenumber):
        """
        Helperfunction
        returns true if current paragraph is finished by detecting end of file or new heading
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
        while (not re.search(pattern, self.lines[linenumber]) and linenumber < len(self.lines) and not self.next_node(linenumber)
               and not self.paragraph_finished(linenumber+1)):
            if not self.lines[linenumber].strip(" ").strip("\n") == "":
                description += self.lines[linenumber]
            linenumber +=1

        return linenumber, description

    def parse_node_description(self, linenumber):
        """
        Function to parse node description from markdown file and add them to node
        """
        description = ""
        while not self.paragraph_finished(linenumber) and not self.next_node(linenumber):
           description += self.lines[linenumber]
           linenumber += 1
        self.node.description = description.strip()

        return linenumber

    def parse_params(self, linenumber):
        """
        Function to parse parameter from markdown file and add them to node
        """
        while not self.paragraph_finished(linenumber) and not self.next_node(linenumber):
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
        while not self.paragraph_finished(linenumber)  and not self.next_node(linenumber):
           #extract subscriber topic and msg type
           match = re.search(self.re_subscriber, self.lines[linenumber])
           if match:
               topic = str(match.group(1))
               type = str(match.group(2))
               linenumber +=1
               linenumber, description = self.extract_description(linenumber, self.re_subscriber)
               self.node.add_subscriber(topic, type, description)
           else:
               linenumber += 1
        return linenumber

    def parse_publisher(self, linenumber):
        """
        Function to parse publisher from markdown file and add them to node
        """
        while not self.paragraph_finished(linenumber)  and not self.next_node(linenumber):
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

    def parse_action_clients(self, linenumber):
        """
        Function to parse action clients from markdown file and add them to node
        """
        while not self.paragraph_finished(linenumber)  and not self.next_node(linenumber):
           #extract action clients topic and action type
           match = re.search(self.re_action_clients, self.lines[linenumber])
           if match:
               topic = str(match.group(1))
               type = str(match.group(2))
               linenumber +=1
               linenumber, description = self.extract_description(linenumber, self.re_action_clients)
               self.node.add_action_client(topic, type, description)
           else:
               linenumber += 1
        return linenumber

    def parse_actions(self, linenumber):
        """
        Function to parse actions from markdown file and add them to node
        """
        while not self.paragraph_finished(linenumber)  and not self.next_node(linenumber):
           #extract action topic and action type
           match = re.search(self.re_actions, self.lines[linenumber])
           if match:
               topic = str(match.group(1))
               type = str(match.group(2))
               linenumber +=1
               linenumber, description = self.extract_description(linenumber, self.re_actions)
               self.node.add_action(topic, type, description)
           else:
               linenumber += 1
        return linenumber

    def parse_service_clients(self, linenumber):
        """
        Function to parse service clients from markdown file and add them to node
        """
        while not self.paragraph_finished(linenumber)  and not self.next_node(linenumber):
           #extract service clients topic and service type
           match = re.search(self.re_service_clients, self.lines[linenumber])
           if match:
               topic = str(match.group(1))
               type = str(match.group(2))
               linenumber +=1
               linenumber, description = self.extract_description(linenumber, self.re_service_clients)
               self.node.add_service_client(topic, type, description)
           else:
               linenumber += 1
        return linenumber

    def parse_services(self, linenumber):
        """
        Function to parse services from markdown file and add them to node
        """
        while not self.paragraph_finished(linenumber) and not self.next_node(linenumber):
           #extract service name and service type
           match = re.search(self.re_services, self.lines[linenumber])
           if match:
               name = str(match.group(1))
               type = str(match.group(2))
               linenumber +=1
               linenumber, description = self.extract_description(linenumber, self.re_services)
               self.node.add_service(name, type, description)
           else:
               linenumber += 1
        return linenumber

    def next_node(self, linenumber):
        next_node = re.search("(<!--) starting node (\S+)", self.lines[linenumber])
        return next_node




