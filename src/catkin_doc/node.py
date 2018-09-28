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
        self.parameters[parameter_name] = (default_value, comment)
        return True

    def add_subscriber(self, topic, msg_type, comment):
        """
        Function to add subscriber to node representation
        """
        self.subscriber[topic] = (msg_type, comment)
        return True

    def add_publisher(self, topic, msg_type, comment):
        """
        Function to add publisher to node representation
        """
        self.publisher[topic] = (msg_type, comment)
        return True

    def add_action_client(self, action_server, action, comment):
        """
        function to add action clients to node representation
        """
        self.action_clients[action_server] = (action, comment)
        return True

    def add_service_client(self, service_topic, srv_type, comment):
        """
        function to add service clients to node representations
        """
        self.service_clients[service_topic] = (srv_type, comment)
        return True

    def add_service(self, name, srv_type, comment):
        """
        function to add services to node representation
        """
        self.services[name] = (srv_type, comment)
        return True

    def add_action(self, name, act_type, comment):
        """
        Function to add actions to node representation
        """
        self.actions[name] = (act_type, comment)
        return True



