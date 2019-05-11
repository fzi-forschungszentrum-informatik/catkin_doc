"""
Node datastructure
"""

import catkin_doc.datastructures as ds


class Node(ds.DocObject):
    """Datastructure representing a node"""

    def add_parameter(self, parameter):
        self.add_child(ds.KEYS[ds.Parameter], parameter)

    def add_subscriber(self, subscriber):
        self.add_child(ds.KEYS[ds.Subscriber], subscriber)

    def add_publisher(self, publisher):
        self.add_child(ds.KEYS[ds.Publisher], publisher)

    def add_service(self, service):
        self.add_child(ds.KEYS[ds.Service], service)

    def add_service_client(self, service_client):
        self.add_child(ds.KEYS[ds.ServiceClient], service_client)

    def add_action(self, action):
        self.add_child(ds.KEYS[ds.Action], action)

    def add_action_client(self, action_client):
        self.add_child(ds.KEYS[ds.ActionClient], action_client)

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

        for key in ds.KEYS.values():
            if key in self.children:
                out_str += formatter.heading(level + 1, key)
                for item in self.children[key]:
                    list_str = item.to_string(level + 2, formatter)
                    out_str += formatter.as_list_item(0, list_str) + formatter.new_line()

        return out_str
