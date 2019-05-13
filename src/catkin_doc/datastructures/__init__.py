"""
Factory for creating DocObjects from a string
"""



KEYS = {"package": "Packages",
        "node": "Nodes",
        "parameter": "Parameters",
        "service": "Advertised Services",
        "service_client": "Service Clients",
        "publisher": "Published topics",
        "subscriber": "Subscribed topics",
        "action": "Actions",
        "action_client": "Action Clients",
        "unknown": "unknown"
        }


def create_doc_object(key):
    """Returns a class matching the given string"""
    if key == "Packages":
        from catkin_doc.datastructures.package import Package
        return Package
    if key == "Parameters":
        from catkin_doc.datastructures.parameter import Parameter
        return Parameter
    if key == "Nodes":
        from catkin_doc.datastructures.node import Node
        return Node
    if key == "Advertised Services":
        from catkin_doc.datastructures.service import Service
        return Service
    if key == "Service Clients":
        from catkin_doc.datastructures.service import ServiceClient
        return ServiceClient
    if key == "Published topics":
        from catkin_doc.datastructures.topic import Publisher
        return Publisher
    if key == "Subscribed topics":
        from catkin_doc.datastructures.topic import Subscriber
        return Subscriber
    if key == "Actions":
        from catkin_doc.datastructures.action import Action
        return Action
    if key == "Action Clients":
        from catkin_doc.datastructures.action import ActionClient
        return ActionClient
    else:
        from catkin_doc.datastructures.doc_object import DocObject
        return DocObject

def get_identifier_for_type(typeid):
    from catkin_doc.datastructures.node import Node
    from catkin_doc.datastructures.parameter import Parameter
    from catkin_doc.datastructures.package import Package
    from catkin_doc.datastructures.service import Service, ServiceClient
    from catkin_doc.datastructures.action import Action, ActionClient
    from catkin_doc.datastructures.topic import Topic, Subscriber, Publisher
    if typeid == Package:
        return "package"
    elif typeid == Node:
        return "node"
    elif typeid == Parameter:
        return "parameter"
    elif typeid == Service:
        return "service"
    elif typeid == ServiceClient:
        return "service_client"
    elif typeid == Publisher:
        return "publisher"
    elif typeid == Subscriber:
        return "subscriber"
    elif typeid == Action:
        return "action"
    elif typeid == ActionClient:
        return "action_client"
    else:
        return "unknown"
