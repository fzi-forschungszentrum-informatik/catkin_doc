"""Node representation"""

class Node(object):
    """An abstract representation for a ROS node"""
    def __init__(self, name):
        self.filename = name
        self._parameters = dict()
        self._subscriber = dict()
        self._publisher = dict()
        self._action_clients = dict()
        self._service_clients = dict()
        self._services = dict()
        self._actions = dict()


    def add_parameter(self, parameter_name, default_value, comment):
        """
        Function to add parameters to the node representation
        """
        self._parameters[parameter_name] = (default_value, comment)
        return True

    def add_subscriber(self, topic, msg_type, comment):
        """
        Function to add subscriber to node representation
        """
        self._subscriber[topic] = (msg_type, comment)
        return True

    def add_publisher(self, topic, msg_type, comment):
        """
        Function to add publisher to node representation
        """
        self._publisher[topic] = (msg_type, comment)
        return True

    def add_action_client(self, action_server, action, comment):
        """
        function to add action clients to node representation
        """
        self._action_clients[action_server] = (action, comment)
        return True

    def add_service_client(self, service_topic, srv_type, comment):
        """
        function to add service clients to node representations
        """
        self._service_clients[service_topic] = (srv_type, comment)
        return True

    def add_service(self, name, srv_type, comment):
        """
        function to add services to node representation
        """
        self._services[name] = (srv_type, comment)
        return True

    def add_action(self, name, act_type, comment):
        """
        Function to add actions to node representation
        """
        self._actions[name] = (act_type, comment)
        return True

    def parameters_to_md(self):
        """
        Generates a markdowntext from parameters
        """
        md = "## Parameters\n"
        md += "<dl>\n"
        for param in self._parameters:
            default_value, comment = self._parameters[param]
            md += "  <dt>" + param
            if self._parameters[param] != "None":
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
        for topic in self._subscriber:
            msg_type, comment = self._subscriber[topic]
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
        for topic in self._publisher:
            msg_type, comment = self._publisher[topic]
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
        for action_server in self._action_clients:
            msg_type, comment = self._action_clients[action_server]
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
        for action in self._actions:
            msg_type, comment = self._actions[action]
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
        for service in self._service_clients:
            msg_type, comment = self._service_clients[service]
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
        for service in self._services:
            msg_type, comment = self._services[service]
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

        md = "# " + self.filename + "\n \n "

        md += params + subs + pubs + action_clients + service_clients + srvs + actions
        return md


    def node_to_md_file(self):
        """
        Generates a  md file from noderepresentation
        """
        self._file = open("README.md", "w+")
        md = self.node_to_md()
        self._file.write(md)
        self._file.close()

