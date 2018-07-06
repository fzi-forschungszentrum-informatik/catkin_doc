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
        md = "## Parameter\n"
        for param in self._parameters:
          md = md + param + ": "
          if self._parameters[param]:
            md = md + "default:" + self._parameters[param] + "\n"
        return md

    def subscriber_to_md(self):
        sub = "## Subscriber\n"
        for topic in self._subscriber:
          sub = sub + "topic: "+ topic + " msg-type: " + self._subscriber[topic] + "\n"
        return sub

    def publisher_to_md(self):
        pub = "## Publisher\n"
        for topic in self._publisher:
          pub = pub + "topic: "+ topic + " msg-type: " + self._publisher[topic] + "\n"
        return pub

    def action_clients_to_md(self):
        client = "## Action Clients\n"
        for action_server in self._action_clients:
            client = client + "Action client for action topic: " + action_server + " and action " + self._action_clients[action_server] + "\n"
        return client

    def action_to_md(self):
        act = "## Actions\n"
        for action in self._actions:
            act = act + "Action: " + action + " and action-type: " + self._actions[action] + "\n"
        return act

    def service_clients_to_md(self):
        client = "## Service Client\n"
        for service in self._service_clients:
            client = client + "service: " + service + " used service-type: "  + self._service_clients[service] + "\n"
        return client

    def service_to_md(self):
        srv = "## Services\n"
        for service in self._services:
            srv = srv + "Name: " + service + " srv-type: " + self._services[service] + "\n"
        return srv

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

