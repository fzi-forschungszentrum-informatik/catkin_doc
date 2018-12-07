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
        self.parameters[parameter_name] = (default_value, comment)
        return True

    def add_subscriber(self, topic, msg_type, comment):
        """
        Function to add subscriber to node representation
        """
        topic = topic.strip(" ")
        msg_type = msg_type.strip(" ")
        comment = comment.strip(" ")
        self.subscriber[topic] = (msg_type, comment)
        return True

    def add_publisher(self, topic, msg_type, comment):
        """
        Function to add publisher to node representation
        """
        topic = topic.strip(" ")
        msg_type = msg_type.strip(" ")
        comment = comment.strip(" ")
        self.publisher[topic] = (msg_type, comment)
        return True

    def add_action_client(self, action_server, action, comment):
        """
        function to add action clients to node representation
        """
        action_server = action_server.strip(" ")
        action = action.strip(" ")
        comment = comment.strip(" ")
        self.action_clients[action_server] = (action, comment)
        return True

    def add_service_client(self, topic, srv_type, comment):
        """
        function to add service clients to node representations
        """
        topic = topic.strip(" ")
        srv_type = srv_type.strip(" ")
        comment = comment.strip(" ")
        self.service_clients[topic] = (srv_type, comment)
        return True

    def add_service(self, topic, srv_type, comment):
        """
        function to add services to node representation
        """
        topic = topic.strip(" ")
        srv_type = srv_type.strip(" ")
        comment = comment.strip(" ")
        self.services[topic] = (srv_type, comment)
        return True

    def add_action(self, name, act_type, comment):
        """
        Function to add actions to node representation
        """
        name = name.strip(" ")
        act_type = act_type.strip(" ")
        comment = comment.strip(" ")
        self.actions[name] = (act_type, comment)
        return True



