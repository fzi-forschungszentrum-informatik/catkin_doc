"""
Node datastructure
"""

from catkin_doc.datastructures.doc_object import DocObject


class Node(DocObject):
    """Datastructure representing a node"""

    parameters_key = "Parameters"
    subscribers_key = "Subscribers"
    publishers_key = "Publishers"
    services_key = "Advertised Services"
    service_clients_key = "Called Services"
    actions_key = "Action Servers"
    action_clients_key = "Actions Clients"

    def add_parameter(self, parameter):
        self.add_child(self.parameters_key, parameter)

    def add_subscriber(self, subscriber):
        self.add_child(self.subscribers_key, subscriber)

    def add_publisher(self, publisher):
        self.add_child(self.publishers_key, publisher)

    def add_service(self, service):
        self.add_child(self.services_key, service)

    def add_service_client(self, service_client):
        self.add_child(self.service_clients_key, service_client)

    def add_action(self, action):
        self.add_child(self.actions_key, action)

    def add_action_client(self, action_client):
        self.add_child(self.action_clients_key, action_client)

    def to_string(self, level, formatter):
        """
        Formats the object as text

        :param int level: Level of heading hierarchy
        :param formatter: Formatter to use
        :return: A formatted string for this object formatted by the given formatter
        :rtype: str
        """

        out_str = formatter.heading(level, self.name) + formatter.new_line()
        out_str += formatter.text(self.description) + formatter.new_line()

        keys_list = [self.parameters_key, self.subscribers_key, self.publishers_key,
                     self.services_key, self.service_clients_key, self.actions_key,
                     self.action_clients_key]
        for key in keys_list:
            if key in self.children:
                out_str += formatter.heading(level + 1, key)
                for item in self.children[key]:
                    list_str = item.to_string(level + 2, formatter)
                    out_str += formatter.as_list_item(0, list_str) + formatter.new_line()

        return out_str
