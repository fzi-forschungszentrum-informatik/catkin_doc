"""
Factory for creating DocObjects from a string
"""

from catkin_doc.datastructures.doc_object import DocObject
from catkin_doc.datastructures.package import Package
from catkin_doc.datastructures.node import Node
from catkin_doc.datastructures.parameter import Parameter
from catkin_doc.datastructures.service import Service, ServiceClient
from catkin_doc.datastructures.action import Action, ActionClient
from catkin_doc.datastructures.topic import Topic, Subscriber, Publisher


KEYS = {Package: "Packages",
        Node: "Nodes",
        Parameter: "Parameters",
        Service: "Advertised Services",
        ServiceClient: "Service Clients",
        Publisher: "Published topics",
        Subscriber: "Subscribed topics",
        Action: "Actions",
        ActionClient: "Action Clients",
        DocObject: "unknown"
        }

def create_doc_object(key):
    """Returns a class matching the given string"""
    return next((typeid for typeid, id_str in KEYS.items() if id_str == key), DocObject)

