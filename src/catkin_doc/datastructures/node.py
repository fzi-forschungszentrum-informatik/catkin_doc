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
Node datastructure
"""

import catkin_doc.datastructures as ds
from catkin_doc.datastructures.doc_object import DocObject


class Node(DocObject):
    """Datastructure representing a node"""

    def add_parameter(self, parameter):
        """Adds a parameter as child"""
        self.add_child(ds.KEYS["parameter"], parameter)

    def add_subscriber(self, subscriber):
        """Adds a subscriber as child"""
        self.add_child(ds.KEYS["subscriber"], subscriber)

    def add_publisher(self, publisher):
        """Adds a publisher as child"""
        self.add_child(ds.KEYS["publisher"], publisher)

    def add_service(self, service):
        """Adds a service as child"""
        self.add_child(ds.KEYS["service"], service)

    def add_service_client(self, service_client):
        """Adds a service client as child"""
        self.add_child(ds.KEYS["service_client"], service_client)

    def add_action(self, action):
        """Adds an action as child"""
        self.add_child(ds.KEYS["action"], action)

    def add_action_client(self, action_client):
        """Adds an action_client as child"""
        self.add_child(ds.KEYS["action_client"], action_client)
