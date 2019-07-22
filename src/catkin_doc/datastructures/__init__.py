# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright (c) 2019, FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions
#    and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of
#    conditions and the following disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# -- END LICENSE BLOCK ------------------------------------------------

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
        "launchfile": "Launchfiles",
        "launch_argument": "Arguments"
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
    if key == "Arguments":
        from catkin_doc.datastructures.parameter import LaunchArgument
        return LaunchArgument
    if key == "Launchfiles":
        from catkin_doc.datastructures.launchfile import LaunchFile
        return LaunchFile
    from catkin_doc.datastructures.doc_object import DocObject
    return DocObject


def get_identifier_for_type(typeid):
    """Get the name key for a given datatype"""
    from catkin_doc.datastructures.node import Node
    from catkin_doc.datastructures.parameter import Parameter, LaunchArgument
    from catkin_doc.datastructures.package import Package
    from catkin_doc.datastructures.service import Service, ServiceClient
    from catkin_doc.datastructures.action import Action, ActionClient
    from catkin_doc.datastructures.topic import Subscriber, Publisher
    from catkin_doc.datastructures.launchfile import LaunchFile
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
    elif typeid == LaunchArgument:
        return "launch_argument"
    elif typeid == LaunchFile:
        return "launchfile"
    return "unknown"
