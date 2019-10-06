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
Parameter datastructure
"""

from catkin_doc.datastructures.doc_object import DocObject


class Parameter(DocObject):
    """Datastructure representing a node's parameter"""

    def __init__(self, name, description="", datatype="", default_value=None, var_name=None):
        super(Parameter, self).__init__(name, description, var_name)
        self.datatype = datatype
        self.default_value = default_value

    def to_string(self, level, formatter):
        """
        Formats the object as text

        :param int level: Level of heading hierarchy
        :param formatter: Formatter to use
        :return: A formatted string for this object formatted by the given formatter
        :rtype: str
        """

        out_str = ""
        if self.var_name:
            out_str += "\"Symbol: " + formatter.bold(self.name) + "\""
        else:
            out_str += "\"" + formatter.bold(self.name) + "\""
        if self.default_value is not None:
            default_formatted = self.default_value
            if isinstance(self.default_value, str):
                default_formatted = "\"{}\"".format(self.default_value)
            out_str += formatter.text(" (default: {})".format(default_formatted))
        else:
            out_str += formatter.text(" (Required)")
        out_str += formatter.new_line()

        out_str += formatter.text(self.get_description())

        return out_str


class LaunchArgument(Parameter):
    """Datastructure representing argument of a launchfile"""

    def __init__(self, name, description="", datatype="", default_value=None, var_name=None):
        super(LaunchArgument, self).__init__(name, description, datatype, default_value, var_name)
        self.default_description = "Please add description. See file \"{}\"."
        self.default_desc_regex = "\s+".join(
            self.default_description.format("(.*)", "(.*)").split())

    def get_description(self):
        """Returns the description or a hint if possible"""
        if self.description:
            return self.description

        if self.line_number:
            return self.default_description.format(self.filename, self.code)

        raise RuntimeError
