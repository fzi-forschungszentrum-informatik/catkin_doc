"""
Action datastructure
"""

from catkin_doc.datastructures.topic import Topic


class Action(Topic):
    """Datastructure representing a Action server"""
    def __init__(self, name, description="", datatype=""):
        super(Action, self).__init__(name, description, datatype)
        self.type_doc_url_base = "http://docs.ros.org/api/{}/html/action/{}.html"
        self.url = self.create_url(datatype)

class ActionClient(Topic):
    """Datastructure representing a Action client"""
    def __init__(self, name, description="", datatype=""):
        super(ActionClient, self).__init__(name, description, datatype)
        self.type_doc_url_base = "http://docs.ros.org/api/{}/html/action/{}.html"
        self.url = self.create_url(datatype)
