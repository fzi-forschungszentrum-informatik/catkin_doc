"""
Service datastructure
"""

from catkin_doc.datastructures.topic import Topic


class Service(Topic):
    """Datastructure representing a Service server"""
    def __init__(self, name, description="", datatype=""):
        super(Service, self).__init__(name, description, datatype)
        self.type_doc_url_base = "http://docs.ros.org/api/{}/html/srv/{}.html"
        self.url = self.create_url(datatype)


class ServiceClient(Topic):
    """Datastructure representing a Service client"""
    def __init__(self, name, description="", datatype=""):
        super(ServiceClient, self).__init__(name, description, datatype)
        self.type_doc_url_base = "http://docs.ros.org/api/{}/html/srv/{}.html"
        self.url = self.create_url(datatype)
