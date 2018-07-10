"""Node representation"""

class Node(object):
    """An abstract representation for a ROS node"""
    def __init__(self):
        self._parameters = dict()
        self._subscriber = dict()
        self._publisher = dict()
        self._action_clients = dict()
        self._service_clients = dict()
        self._services = dict()
        self._actions = dict()
        self._file = None

    def add_parameter(self, parameter_name, default_value=None):
        self._parameters[parameter_name] = default_value

    def add_subscriber(self, topic, msg_type):
        self._subscriber[topic] = msg_type

    def add_publisher(self, topic, msg_type):
        self._publisher[topic] = msg_type

    def add_action_client(self, action_server, action):
        self._action_clients[action_server] = action

    def add_service_client(self,service_topic, srv_type):
        self._service_clients[service_topic] = srv_type

    def add_service(self, name, type):
        self._services[name] = type

    def add_action(self, name, type):
        self._actions[name] = type

    def parameters_to_md(self):
        md = "## Parameters\n"
        md += "<dl>\n"
        for param in self._parameters:
          md += "  <dt>" + param
          if self._parameters[param]:
            md += "( default : " + self._parameters[param] + ")"
          md += "</dt>\n  <dd>Please add description here.</dd>\n"
        md += "</dl>\n \n"
        return md

    def subscriber_to_md(self):
        md = "## Subscribed Topics\n"
        md += "<dl>\n"
        for topic in self._subscriber:
          md += "  <dt>" + topic + " (" + self._subscriber[topic] + ") </dt>\n"
          md += "  <dd>Please add description here.</dd>\n"
        md += "</dl>\n \n"
        return md

    def publisher_to_md(self):
        md = "## Published Topics\n"
        md += "<dl>\n"
        for topic in self._publisher:
          md += "  <dt>" + topic + " (" + self._publisher[topic] + ") </dt>\n"
          md += "  <dd>Please add description here.</dd>\n"
        md += "</dl>\n \n"
        return md

    def action_clients_to_md(self):
        client = "## Action Clients\n"
        client += "<dl>\n"
        for action_server in self._action_clients:
            client += "  <dt>" + action_server + " (" + self._action_clients[action_server] + ") </dt>\n"
            client += "  <dd>Please add description here.</dd>\n"
        client += "</dl>\n \n"
        return client

    def action_to_md(self):
        md = "## Actions\n"
        md += "<dl>\n"
        for action in self._actions:
            md += "  <dt>" + action + " (" + self._actions[action] + ") </dt>\n"
            md += "  <dd>Please add description here.</dd>\n"
        md += "</dl>\n \n"
        return md

    def service_clients_to_md(self):
        md = "## Service Clients\n"
        md += "<dl>\n"
        for service in self._service_clients:
            md += "  <dt>" + service + " ("  + self._service_clients[service] + ") </dt>\n"
            md += "  <dd>Please add description here.</dd>\n"
        md += "</dl>\n \n"
        return md

    def service_to_md(self):
        md = "## Services\n"
        md += "<dl>\n"
        for service in self._services:
            md += "  <dt>" + service + " ("  + self._services[service] + ") </dt>\n"
            md += "  <dd>Please add description here.</dd>\n"
        md += "</dl>\n \n"
        return md

    def node_to_md(self):
        params = self.parameters_to_md()
        subs = self.subscriber_to_md()
        pubs = self.publisher_to_md()
        action_clients = self.action_clients_to_md()
        service_clients = self.service_clients_to_md()
        srvs = self.service_to_md()
        actions = self.action_to_md()

        md = params + subs + pubs + action_clients + service_clients + srvs + actions
        return md


    def node_to_md_file(self):
        self._file = open("README.md", "w+")
        md = self.node_to_md()
        self._file.write(md)
        self._file.close()

