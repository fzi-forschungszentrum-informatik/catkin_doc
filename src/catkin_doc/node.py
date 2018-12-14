"""Node representation"""

class Node(object):
    """An abstract representation for a ROS node"""
    def __init__(self, name):
        self.filename = name
        self.parameters = dict()
        self.subscriber = dict()
        self.publisher = dict()
        self.action_clients = dict()
        self.service_clients = dict()
        self.services = dict()
        self.actions = dict()


    def add_parameter(self, parameter_name, default_value, comment):
        """
        Function to add parameters to the node representation
        """
        parameter_name = parameter_name.strip(" ")
        if default_value is not None:
            default_value = default_value.strip(" ")
        comment = comment.strip(" ")
        if parameter_name in self.parameters:
            oldmsg, oldcom = self.parameters[parameter_name]
            if "Please add description" not in oldcom and "Please add description" in comment:
                comment = oldcom
        self.parameters[parameter_name] = (default_value, comment)
        return True

    def add_subscriber(self, topic, msg_type, comment):
        """
        Function to add subscriber to node representation
        """
        topic = topic.strip(" ")
        msg_type = msg_type.strip(" ")
        comment = comment.strip(" ")
        if topic in self.subscriber:
            oldmsg, oldcom = self.subscriber[topic]
            if "Please add description" not in oldcom and "Please add description" in comment:
                comment = oldcom
        self.subscriber[topic] = (msg_type, comment)
        return True

    def add_publisher(self, topic, msg_type, comment):
        """
        Function to add publisher to node representation
        """
        topic = topic.strip(" ")
        msg_type = msg_type.strip(" ")
        comment = comment.strip(" ")
        if topic in self.publisher:
            oldmsg, oldcom = self.publisher[topic]
            if "Please add description" not in oldcom and "Please add description" in comment:
                comment = oldcom
        self.publisher[topic] = (msg_type, comment)
        return True

    def add_action_client(self, action_server, action, comment):
        """
        function to add action clients to node representation
        """
        action_server = action_server.strip(" ")
        action = action.strip(" ")
        comment = comment.strip(" ")
        if action_server in self.action_clients:
            oldmsg, oldcom = self.action_clients[action_server]
            if "Please add description" not in oldcom and "Please add description" in comment:
                comment = oldcom
        self.action_clients[action_server] = (action, comment)
        return True

    def add_service_client(self, topic, srv_type, comment):
        """
        function to add service clients to node representations
        """
        topic = topic.strip(" ")
        srv_type = srv_type.strip(" ")
        comment = comment.strip(" ")
        if topic in self.service_clients:
            oldmsg, oldcom = self.service_clients[topic]
            if "Please add description" not in oldcom and "Please add description" in comment:
                comment = oldcom
        self.service_clients[topic] = (srv_type, comment)
        return True

    def add_service(self, topic, srv_type, comment):
        """
        function to add services to node representation
        """
        topic = topic.strip(" ")
        srv_type = srv_type.strip(" ")
        comment = comment.strip(" ")
        if topic in self.services:
            oldmsg, oldcom = self.services[topic]
            if "Please add description" not in oldcom and "Please add description" in comment:
                comment = oldcom
        self.services[topic] = (srv_type, comment)
        return True

    def add_action(self, name, act_type, comment):
        """
        Function to add actions to node representation
        """
        name = name.strip(" ")
        act_type = act_type.strip(" ")
        comment = comment.strip(" ")
        if name in self.actions:
            oldmsg, oldcom = self.actions[name]
            if "Please add description" not in oldcom and "Please add description" in comment:
                comment = oldcom
        self.actions[name] = (act_type, comment)
        return True



