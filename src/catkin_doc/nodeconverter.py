import urllib2
"""class to convert node representation to documentation file"""
class NodeConverter(object):
    def __init__(self):
        self.node = None
        pass

    def convert_to_file(self, node, filetype, file_name):
        """
        function to convert given node to file of type filetype where filetype may be md or rst
        """
        self.file_name = file_name
        self.node = node
        if filetype == "md":
            self.node_to_md_file()
        elif filetype == "rst":
            self.node_to_rst_file()
        else:
            print("Please use one of the filetypes 'md' or 'rst'!")


    def md_definition(self, name, type, comment):
        """
        Helper function to create markdown
        """
        md = "   * **" + name.strip(" ") + "** ("
        if type != "":
            md += type + ")\n \n"
        else:
            md += "Add type here)\n \n"
        if comment != "":
            md += "      " + comment + "  \n"
        else:
            md += "      Please add description here.\n  "
        return md


    def parameters_to_md(self):
        """
        Generates a markdowntext from parameters
        """
        md = "## Parameters\n"
        md += ""
        for param in sorted(self.node.parameters.iterkeys()):
            default_value, comment = self.node.parameters[param]
            if default_value is not None:
                value = "default: " + default_value
            else:
                value = "default: -"
            md += self.md_definition(param, value, comment)

        md += "\n \n"
        return md

    def subscriber_to_md(self):
        """
        Generates a markdown text from subscribers
        """
        md = "## Subscribed Topics\n"
        md += ""
        for topic in sorted(self.node.subscriber.iterkeys()):
            msg_type, comment = self.node.subscriber[topic]
            if not 'fzi' in msg_type:
               types = msg_type.split('::')
               if len(types) >= 2:
                   types[1] = types[1].strip('Action')
                   url = "http://docs.ros.org/kinetic/api/" + types[0] +"/html/msg/" + types[1] + ".html"
                   if self.check_url(url):
                      msg_type ="[" + msg_type + "](" + url + ")"
            md += self.md_definition(topic, msg_type, comment)
        md += "\n \n"
        return md

    def publisher_to_md(self):
        """
        Generates a markdown text from publishers
        """
        md = "## Advertised Topics\n"
        md += ""
        for topic in sorted(self.node.publisher.iterkeys()):
            msg_type, comment = self.node.publisher[topic]
            if not 'fzi' in msg_type:
               types = msg_type.split('::')
               if len(types) >= 2:
                   types[1] = types[1].strip('Action')
                   url = "http://docs.ros.org/kinetic/api/" + types[0] +"/html/msg/" + types[1] + ".html"
                   if self.check_url(url):
                      msg_type ="[" + msg_type + "](" + url + ")"
            md += self.md_definition(topic, msg_type, comment)
        md += "\n \n"
        return md

    def action_clients_to_md(self):
        """
        Generates a markdowntext from action clients
        """
        md = "## Action clients\n"
        md += ""
        for action_server in sorted(self.node.action_clients.iterkeys()):
            msg_type, comment = self.node.action_clients[action_server]
            if not 'fzi' in msg_type:
               types = msg_type.split('::')
               if len(types) >= 2:
                   types[1] = types[1].strip('Action')
                   url = "http://docs.ros.org/kinetic/api/" + types[0] +"/html/action/" + types[1] + ".html"
                   if self.check_url(url):
                      msg_type ="[" + msg_type + "](" + url + ")"
            md += self.md_definition(action_server, msg_type, comment)
        md += "\n \n"
        return md

    def action_to_md(self):
        """
        Generates a markdowntext from actions
        """
        md = "## Action servers\n"
        md += ""
        for action in sorted(self.node.actions.iterkeys()):
            msg_type, comment = self.node.actions[action]
            if not 'fzi' in msg_type:
               types = msg_type.split('::')
               if len(types) >= 2:
                   types[1] = types[1].strip('Action')
                   url = "http://docs.ros.org/kinetic/api/" + types[0] +"/html/action/" + types[1] + ".html"
                   if self.check_url(url):
                      msg_type ="[" + msg_type + "](" + url + ")"
            md += self.md_definition(action, msg_type, comment)
        md += "\n \n"
        return md

    def service_clients_to_md(self):
        """
        Generates a markdowntext from service clients
        """
        md = "## Service Clients\n"
        md += ""
        for service in sorted(self.node.service_clients.iterkeys()):
            msg_type, comment = self.node.service_clients[service]
            if not 'fzi' in msg_type:
               types = msg_type.split('::')
               if len(types) >= 2:
                   types[1] = types[1].strip('Action')
                   url = "http://docs.ros.org/kinetic/api/" + types[0] +"/html/srv/" + types[1] + ".html"
                   if self.check_url(url):
                      msg_type ="[" + msg_type + "](" + url + ")"
            md += self.md_definition(service, msg_type, comment)
        md += "\n \n"
        return md

    def service_to_md(self):
        """
        Generates a markdowntext from services
        """
        md = "## Advertised services\n"
        md += ""
        for service in sorted(self.node.services.iterkeys()):
            msg_type, comment = self.node.services[service]
            if not 'fzi' in msg_type:
               types = msg_type.split('::')
               if len(types) >= 2:
                   types[1] = types[1].strip('Action')
                   url = "http://docs.ros.org/kinetic/api/" + types[0] +"/html/srv/" + types[1] + ".html"
                   if self.check_url(url):
                      msg_type ="[" + msg_type + "](" + url + ")"
            md += self.md_definition(service, msg_type, comment)
        md += "\n \n"
        return md

    def node_to_md(self):
        """
        Generates a markdowntext from node represenation
        """
        params = self.parameters_to_md()
        subs = self.subscriber_to_md()
        pubs = self.publisher_to_md()
        action_clients = self.action_clients_to_md()
        service_clients = self.service_clients_to_md()
        srvs = self.service_to_md()
        actions = self.action_to_md()

        md = "\n<!-- starting node " + self.node.filename + " --> \n\n"
        md += "# " + self.node.filename + "\n \n "
        md += "<!-- Please add any additional node description after this comment --> \n"
        md += self.node.description
        md += "\n\n"

        md += params + subs + pubs + action_clients + service_clients + srvs + actions
        return md


    def node_to_md_file(self):
        """
        Generates a  md file from noderepresentation
        """
        file = open(self.file_name, "a+")
        md = self.node_to_md()
        file.write(md)
        file.close()

    def node_to_rst_file(self):
        """
        Generates a rst file from noderepresentation
        """
        file = open(self.file_name, "a+")
        rst = self.node_to_rst()
        file.write(rst)
        file.close

    def node_to_rst(self):
        """
        Generates struktured text from noderepresentation
        """
        params = self.parameters_to_rst()
        subs = self.subscriber_to_rst()
        pubs = self.publisher_to_rst()
        service_clients = self.service_clients_to_rst()
        srvs = self.service_to_rst()
        action_clients = self.action_clients_to_rst()
        actions = self.action_to_rst()

        rst = "\n.. starting node " + self.node.filename + " \n\n"
        rst += self.node.filename + "\n===================================\n\n"
        rst += ".. Please add any additional node description after this comment\n"
        rst += self.node.description
        rst += "\n\n"


        rst += params + subs + pubs + service_clients + srvs + action_clients + actions
        return rst

    def rst_definition(self, name, type, comment):
        """
        Helper function to create rst
        """
        rst = name.strip(' ') + " ("
        if type != "":
            rst += type + ")\n"
        else:
            rst += "Add type here)\n"
        if comment != "":
            rst += "    " + comment + "\n"
        else:
            rst += "    Please add description here\n"
        return rst

    def parameters_to_rst(self):
        """
        Turn params from node representation to rst
        """
        rst = "Parameter \n----------------------------------------------\n"
        for param in sorted(self.node.parameters.iterkeys()):
            default_value, comment = self.node.parameters[param]
            if default_value is not None:
                default_value_rst = "default value: " + default_value
            else:
                default_value_rst = "default value: -"
            rst += self. rst_definition(param, default_value_rst, comment)
        rst += "\n"
        return rst

    def subscriber_to_rst(self):
        """
        Turn subs from node representation to rst
        """
        rst = "Subscribed topics\n---------------------------------------\n"
        for topic in sorted(self.node.subscriber.iterkeys()):
            msg_type, comment = self.node.subscriber[topic]
            if not 'fzi' in msg_type:
               types = msg_type.split('::')
               if len(types) >= 2:
                   types[1] = types[1].strip('Action')
                   url = "http://docs.ros.org/kinetic/api/" + types[0] +"/html/msg/" + types[1] + ".html"
                   if self.check_url(url):
                      msg_type ="`" + msg_type + " <" + url + ">`_"
            rst += self.rst_definition(topic, msg_type, comment)
        rst += "\n"
        return rst

    def publisher_to_rst(self):
        """
        Turn pubs from node representation to rst
        """
        rst = "Advertised topics\n----------------------------------------\n"
        for topic in sorted(self.node.publisher.iterkeys()):
            msg_type, comment = self.node.publisher[topic]
            if not 'fzi' in msg_type:
               types = msg_type.split('::')
               if len(types) >= 2:
                   url = "http://docs.ros.org/kinetic/api/" + types[0] +"/html/msg/" + types[1] + ".html"
                   if self.check_url(url):
                      msg_type ="`" + msg_type + " <" + url + ">`_"
            rst += self.rst_definition(topic, msg_type, comment)
        rst += "\n"
        return rst

    def action_clients_to_rst(self):
        """
        Turn action clients from node representation to rst
        """
        rst = "Action clients\n----------------------------------------\n"
        for action_server in sorted(self.node.action_clients.iterkeys()):
            msg_type, comment = self.node.action_clients[action_server]
            if not 'fzi' in msg_type:
               types = msg_type.split('::')
               if len(types) >= 2:
                   types[1] = types[1].strip('Action')
                   url = "http://docs.ros.org/kinetic/api/" + types[0] +"/html/action/" + types[1] + ".html"
                   if self.check_url(url):
                      msg_type ="`" + msg_type + " <" + url + ">`_"
            rst += self.rst_definition(action_server, msg_type, comment)
        rst += "\n"
        return rst

    def action_to_rst(self):
        """
        Turn action from node representation to rst
        """
        rst = "Action server\n----------------------------------------\n"
        for action in sorted(self.node.actions.iterkeys()):
            msg_type, comment = self.node.actions[action]
            if not 'fzi' in msg_type:
               types = msg_type.split('::')
               if len(types) >= 2:
                   types[1] = types[1].strip('Action')
                   url = "http://docs.ros.org/kinetic/api/" + types[0] +"/html/action/" + types[1] + ".html"
                   if self.check_url(url):
                      msg_type ="`" + msg_type + " <" + url + ">`_"
            rst += self.rst_definition(action, msg_type, comment)
        rst += "\n"
        return rst

    def service_clients_to_rst(self):
        """
        Turn service clients from node representation to rst
        """
        rst =  "Service clients\n----------------------------------------\n"
        for service in sorted(self.node.service_clients.iterkeys()):
            msg_type, comment = self.node.service_clients[service]
            if not 'fzi' in msg_type:
               types = msg_type.split('::')
               if len(types) >= 2:
                   url = "http://docs.ros.org/kinetic/api/" + types[0] +"/html/srv/" + types[1] + ".html"
                   if self.check_url(url):
                      msg_type ="`" + msg_type + " <" + url + ">`_"
            rst += self.rst_definition(service, msg_type, comment)
        rst += "\n"
        return rst

    def service_to_rst(self):
        """
        Turn services from node representation to rst
        """
        rst =  "Advertised services\n----------------------------------------\n"
        for service in sorted(self.node.services.iterkeys()):
            msg_type, comment = self.node.services[service]
            if not 'fzi' in msg_type:
               types = msg_type.split('::')
               if len(types) >= 2:
                   url = "http://docs.ros.org/kinetic/api/" + types[0] +"/html/srv/" + types[1] + ".html"
                   if self.check_url(url):
                      msg_type ="`" + msg_type + " <" + url + ">`_"
            rst += self.rst_definition(service, msg_type, comment)
        rst += "\n"
        return rst

    def check_url(self, url):
        """
        Function to check if url is valid and functioning
        """
        try:
                if not urllib2.urlparse.urlparse(url).netloc:
                    return False

                website = urllib2.urlopen(url)
                html = website.read()

                if website.code != 200 :
                    return False
        except urllib2.URLError, e:
                print "Could not validate link: ", url
                print e
                print "Skipping url"
                return False

        return True



