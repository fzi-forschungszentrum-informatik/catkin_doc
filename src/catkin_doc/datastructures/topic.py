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
Topic datastructure
"""
try:
    from urllib.request import urlopen
    from urllib.error import URLError
except ImportError:
    from urllib2 import urlopen, URLError
from catkin_doc.datastructures.doc_object import DocObject


class Topic(DocObject):
    """Datastructure representing a subscribed or advertised topic"""

    def __init__(self, name, description="", datatype="", var_name=None):
        super(Topic, self).__init__(name, description, var_name)
        self.datatype = datatype
        self.type_doc_url_base = "http://docs.ros.org/api/{}/html/msg/{}.html"
        self.url = None
        self.validate_urls = True # TODO: This should be propagated as a runtime argument
        self.create_url(datatype)

    def to_string(self, level, formatter):
        """
        Formats the object as text

        :param int level: Level of heading hierarchy
        :param formatter: Formatter to use
        :return: A formatted string for this object formatted by the given formatter
        :rtype: str
        """

        out_str = ""
        print_name = "".join(filter(None, [self.namespace, self.name]))
        type_str = self.datatype
        if self.var_name:
            print_name = "Symbol: " + self.name
        if self.url:
            type_str = formatter.link(self.url, self.datatype)
        out_str += formatter.heading(level, print_name + " ({})".format(type_str))
        out_str += self.get_description() + formatter.new_line()

        return out_str

    def create_url(self, datatype):
        """
        Function to create url and check if url is valid and functioning
        :param str datatype: Message type for the given topic
        :return: A functioning url or None if no url is found
        :rtype: str
        """
        types = datatype.split('/')
        if len(types) >= 2 and not 'fzi' in datatype:
            url = self.type_doc_url_base.format(types[0], types[1])
            if self.validate_urls:
                try:
                    website = urlopen(url)

                    if website.code == 200:
                        self.url = url
                except URLError:
                    # print "Could not validate link: ", url
                    # print e
                    # print "Skipping url"
                    pass
            else:
                self.url = url


class Subscriber(Topic):
    """Datastructure representing a Subscriber"""
class Publisher(Topic):
    """Datastructure representing a Publisher"""
