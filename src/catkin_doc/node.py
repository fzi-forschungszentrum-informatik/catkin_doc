"""Node representation"""

class Node(object):
    """An abstract representation for a ROS node"""
    def __init__(self):
        self._parameters = dict()
        self._subscriber = dict()
        self._publisher = dict()
        self._action_clients = dict()
        self._file = None

    def add_parameter(self, parameter_name, default_value=None):
        self._parameters[parameter_name] = default_value

    def add_subscriber(self, topic, msg_type):
        self._subscriber[topic] = msg_type

    def add_publisher(self, topic, msg_type):
        self._publisher[topic] = msg_type

    def add_action_client(self, action_server, action):
        self._action_clients[action_server] = action


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
            client = client + "Action client for action topic: " + action_server + " and action" + self._action_clients[action_server]
        return client

    def node_to_md(self):
        params = self.parameters_to_md()
        subs = self.subscriber_to_md()
        pubs = self.publisher_to_md()
        action_clients = self.action_clients_to_md()

        md = params + subs + pubs + action_clients

        return md


    def node_to_md_file(self):
        self._file = open("README.md", "w+")
        md = self.node_to_md()
        self._file.write(md)
        self._file.close()

