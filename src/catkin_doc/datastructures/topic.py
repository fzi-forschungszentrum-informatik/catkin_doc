"""
Topic datastructure
"""

from catkin_doc.datastructures.doc_object import DocObject
import urllib2


class Topic(DocObject):
    """Datastructure representing a subscribed or advertised topic"""

    def __init__(self, name, description="", datatype=""):
        super(Topic, self).__init__(name, description)
        self.datatype = datatype
        self.url = self.create_url(datatype)

    def to_string(self, level, formatter):
        """
        Formats the object as text

        :param int level: Level of heading hierarchy
        :param formatter: Formatter to use
        :return: A formatted string for this object formatted by the given formatter
        :rtype: str
        """

        out_str = formatter.bold(self.name)
        if self.url:
            out_str += formatter.text(" ({})".format(formatter.link(self.url, self.datatype)))
        else:
            out_str += formatter.text(" ({})".format(self.datatype))
        out_str += formatter.new_line()

        out_str += formatter.text(self.get_description())

        return out_str

    def create_url(self, datatype):
            """
            Function to create url and check if url is valid and functioning
            :param str datatype: Message type for the given topic
            :return: A functioning url or None if no url is found
            :rtype: str
            """
            types = datatype.split('::')
            if len(types) >= 2 and not 'fzi' in datatype:
                if 'Action' in types[1]:
                  types[1] = types[1].strip('Action')
                  url = "http://docs.ros.org/api/" + types[0] +"/html/action/" + types[1] + ".html"
                elif 'Service' in types[1]:
                    types[1] = types[1].strip('Service')
                    url = "http://docs.ros.org/api/" + types[0] +"/html/srv/" + types[1] + ".html"
                elif 'Message' in types[1]:
                    types[1] = types[1]
                    url = "http://docs.ros.org/api/" + types[0] +"/html/msg/" + types[1] + ".html"
                else:
                    return None

                try:
                        if not urllib2.urlparse.urlparse(url).netloc:
                            return False

                        website = urllib2.urlopen(url)
                        html = website.read()

                        if website.code != 200 :
                            return False
                except urllib2.URLError, e:
                        print "Could not validate link: ", url
                        print e
                        print "Skipping url"
                        return None
                return url
            return None


class Subscriber(Topic):
    """Datastructure representing a Subscriber"""
class Publisher(Topic):
    """Datastructure representing a Publisher"""
