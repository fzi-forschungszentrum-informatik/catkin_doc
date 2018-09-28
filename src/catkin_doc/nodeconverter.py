"""class to convert node representation to documentation file"""
class NodeConverter(object):
    def __init__(self):
        self.node = None
        pass

    def convert_to_file(self, node, filetype):
        """
        function to convert given node to file of type filetype where filetype may be md or rst
        """
        self.node = node
        if filetype == "md":
            self.node_to_md_file()
        elif filetype == "rst":
            self.node_to_rst_file()
        else:
            print("Please use one of the filetypes 'md' or 'rst'!")

    def parameters_to_md(self):
        """
        Generates a markdowntext from parameters
        """
        md = "## Parameters\n"
        md += "<dl>\n"
        for param in self.node.parameters:
            default_value, comment = self.node.parameters[param]
            md += "  <dt>" + param
            if default_value is not None:
                md += "( default: " + default_value + ")"
            else:
                md += "( default: - )"

            if comment != "":
                md += "  <dd>" + comment + "</dd>\n"
            else:
                md += "</dt>\n  <dd>Please add description here.</dd>\n"
        md += "</dl>\n \n"
        return md

    def subscriber_to_md(self):
        """
        Generates a markdown text from subscribers
        """
        md = "## Subscribed Topics\n"
        md += "<dl>\n"
        for topic in self.node.subscriber:
            msg_type, comment = self.node.subscriber[topic]
            md += "  <dt>" + topic + " (" + msg_type + ") </dt>\n"
            if comment != "":
                md += "  <dd>" + comment + "</dd>\n"
            else:
                md += "  <dd>Please add description here.</dd>\n"
        md += "</dl>\n \n"
        return md

    def publisher_to_md(self):
        """
        Generates a markdown text from publishers
        """
        md = "## Published Topics\n"
        md += "<dl>\n"
        for topic in self.node.publisher:
            msg_type, comment = self.node.publisher[topic]
            md += "  <dt>" + topic + " (" + msg_type + ") </dt>\n"
            if comment != "":
                md += "  <dd>" + comment + "</dd>\n"
            else:
                md += "  <dd>Please add description here.</dd>\n"
        md += "</dl>\n \n"
        return md

    def action_clients_to_md(self):
        """
        Generates a markdowntext from action clients
        """
        md = "## Action Clients\n"
        md += "<dl>\n"
        for action_server in self.node.action_clients:
            msg_type, comment = self.node.action_clients[action_server]
            md += "  <dt>" + action_server + " (" + msg_type + ") </dt>\n"
            if comment != "":
                md += "  <dd>" + comment + "</dd>\n"
            else:
                md += "  <dd>Please add description here.</dd>\n"
        md += "</dl>\n \n"
        return md

    def action_to_md(self):
        """
        Generates a markdowntext from actions
        """
        md = "## Actions\n"
        md += "<dl>\n"
        for action in self.node.actions:
            msg_type, comment = self.node.actions[action]
            md += "  <dt>" + action + " (" + msg_type + ") </dt>\n"
            if comment != "":
                md += "  <dd>" + comment + "</dd>\n"
            else:
                md += "  <dd>Please add description here.</dd>\n"
        md += "</dl>\n \n"
        return md

    def service_clients_to_md(self):
        """
        Generates a markdowntext from service clients
        """
        md = "## Service Clients\n"
        md += "<dl>\n"
        for service in self.node.service_clients:
            msg_type, comment = self.node.service_clients[service]
            md += "  <dt>" + service + " ("  + msg_type + ") </dt>\n"
            if comment != "":
                md += "  <dd>" + comment + "</dd>\n"
            else:
                md += "  <dd>Please add description here.</dd>\n"
        md += "</dl>\n \n"
        return md

    def service_to_md(self):
        """
        Generates a markdowntext from services
        """
        md = "## Services\n"
        md += "<dl>\n"
        for service in self.node.services:
            msg_type, comment = self.node.services[service]
            md += "  <dt>" + service + " ("  + msg_type + ") </dt>\n"
            if comment != "":
                md += "  <dd>" + comment + "</dd>\n"
            else:
                md += "  <dd>Please add description here.</dd>\n"
        md += "</dl>\n \n"
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

        md = "# " + self.node.filename + "\n \n "

        md += params + subs + pubs + action_clients + service_clients + srvs + actions
        return md


    def node_to_md_file(self):
        """
        Generates a  md file from noderepresentation
        """
        filename = self.node.filename + ".md"
        file = open(filename, "w+")
        md = self.node_to_md()
        file.write(md)
        file.close()

    def node_to_rst_file(self):
        """
        Generates a rst file from noderepresentation
        """
        filename = self.node.filename + ".rst"
        file = open(filename, "w+")
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
        action_clients = self.action_clients_to_rst()
        service_clients = self.service_clients_to_rst()
        srvs = self.service_to_rst()
        actions = self.action_to_rst()

        rst = "# " + self.node.filename + "\n ===================================\n "

        rst += params + subs + pubs + action_clients + service_clients + srvs + actions
        return rst

    def rst_definition(self, name, type, comment):
        """
        Helper function to create rst
        """
        rst = name + " ("
        if type != "":
            rst += type + ")\n"
        else:
            rst += "Add type here)\n"
        if comment != "":
            rst += "\t" + comment
        else:
            rst += "\tPlease add description here"
        return rst

    def parameters_to_rst(self):
        """
        Turn params from node representation to rst
        """
        rst = "Parameter \n----------------------------------------------\n"
        for param in self.node.parameters:
            default_value, comment = self.node.parameters[param]
            default_value_rst = "default value: " + default_value
            rst += self. rst_definition(param, default_value_rst, comment)
        return rst

    def subscriber_to_rst(self):
        """
        Turn subs from node representation to rst
        """
        rst = "Subscribed Topics\n---------------------------------------\n"
        for topic in self.node.subscriber:
            msg_type, comment = self.node.subscriber[topic]
            rst += self. rst_definition(topic, msg_type, comment)
        return rst

    def publisher_to_rst(self):
        """
        Turn pubs from node representation to rst
        """
        rst = "Published Topics\n----------------------------------------\n"
        for topic in self.node.publisher:
            msg_type, comment = self.node.publisher[topic]
            rst += self. rst_definition(topic, msg_type, comment)
        return rst

    def action_clients_to_rst(self):
        """
        Turn action clients from node representation to rst
        """
        rst = "Action Clients\n----------------------------------------\n"
        for action_server in self.node.action_clients:
            msg_type, comment = self.node.action_clients[action_server]
            rst += self. rst_definition(action_server, msg_type, comment)
        return rst

    def action_to_rst(self):
        """
        Turn action from node representation to rst
        """
        rst = "Actions\\n----------------------------------------\n"
        for action in self.node.actions:
            msg_type, comment = self.node.actions[action]
            rst += self. rst_definition(action, msg_type, comment)
        return rst

    def service_clients_to_rst(self):
        """
        Turn service clients from node representation to rst
        """
        rst =  "Service Clients\\n----------------------------------------\n"
        for service in self.node.service_clients:
            msg_type, comment = self.node.service_clients[service]
            rst += self. rst_definition(service, msg_type, comment)
        return rst

    def service_to_rst(self):
        """
        Turn services from node representation to rst
        """
        rst =  "Services\\n----------------------------------------\n"
        for service in self.node.services:
            msg_type, comment = self.node.services[service]
            rst += self. rst_definition(service, msg_type, comment)
        return rst

