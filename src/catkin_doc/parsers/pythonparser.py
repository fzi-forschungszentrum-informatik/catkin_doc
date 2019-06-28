"""Small module to check a python file for ros api items"""

from __future__ import print_function

import ast
import tokenize
import re

from catkin_doc.datastructures.node import Node
from catkin_doc.datastructures.parameter import Parameter
from catkin_doc.datastructures.service import Service, ServiceClient
from catkin_doc.datastructures.action import Action, ActionClient
from catkin_doc.datastructures.topic import Subscriber, Publisher


class Analyzer(ast.NodeVisitor):
    func_names = ["Publisher", "Subscriber", "Service", "ServiceProxy", "SimpleActionClient",
                  "Parameter", "SimpleActionServer", "get_param"]

    def __init__(self, tokens):
        self.stats = {"import": []}
        for name in self.func_names:
            self.stats[name] = list()

        self.tokens = tokens
        self.from_imports = dict()

    def search_comment(self, line_begin):
        comments = list()
        for token in reversed(self.tokens):
            token_line = token[2][0]
            if token_line >= line_begin:
                continue

            if token[0] == tokenize.NL and token[4] != "\n":
                continue
            if token[0] == tokenize.COMMENT:
                comments.append(re.sub(r"^\s*#\s*", "", token[1]))
            else:
                break
        return " ".join(reversed(comments))

    @staticmethod
    def parse_datatype(datatype_string):
        """
        Extracts package/Datatype from package.msg/Datatype strings
        """
        # print("Datatype: " + datatype_string)
        try:
            package, msg_type = datatype_string.split("/")
        except ValueError:
            return datatype_string
        return "/".join([re.sub(r'\.(msg|srv|action)', '', package), msg_type])

    def ast_to_python(self, ast_type):
        """Tries to convert an ast datatype to a standard python type"""
        if isinstance(ast_type, ast.Str):
            python_type = ast_type.s
        elif isinstance(ast_type, ast.Name):
            python_type = "id: " + ast_type.id
        elif isinstance(ast_type, ast.Num):
            python_type = ast_type.n
        elif isinstance(ast_type, ast.List):
            python_type = [self.ast_to_python(x) for x in ast_type.elts]
        elif isinstance(ast_type, ast.Dict):
            python_type = dict()
            for pair in zip(ast_type.keys, ast_type.values):
                python_type[self.ast_to_python(pair[0])] = self.ast_to_python(pair[1])
        else:
            python_type = "UNKNOWN_TYPE"

        return python_type

    def visit_Call(self, node):
        if isinstance(node.func, ast.Attribute):
            func_name = node.func.attr
            if func_name == "get_param":
                param_name = ""
                default_value = ""
                if len(node.args) >= 1:
                    if isinstance(node.args[0], ast.Str):
                        param_name = node.args[0].s
                    elif isinstance(node.args[0], ast.Name):
                        param_name = "id: " + node.args[0].id

                if len(node.args) >= 2:
                    default_value = self.ast_to_python(node.args[1])

                # Search for comment
                comment = self.search_comment(node.lineno)
                self.stats["get_param"].append(
                    {"name": param_name,
                     "lineno": node.lineno,
                     "default": default_value,
                     "comment": comment})
            elif func_name in self.func_names:
                topic_name = ""
                msg_type = ""
                if len(node.args) >= 2:
                    topic_name = self.ast_to_python(node.args[0])

                    if isinstance(node.args[1], ast.Name):
                        if node.args[1].id in self.from_imports:
                            msg_type = self.from_imports[node.args[1].id]
                        else:
                            msg_type = "id: " + node.args[1].id
                    elif isinstance(node.args[1], ast.Attribute):
                        def parse(candidate):
                            if isinstance(candidate, ast.Attribute):
                                return ".".join([parse(candidate.value), candidate.attr])
                            return candidate.id
                        msg_type = "/".join([parse(node.args[1].value), node.args[1].attr])
                        # print(msg_type)
                    else:
                        # I currently see no way of getting here
                        raise RuntimeError("Cannot parse message type type of call {}('{}')"
                                           .format(func_name, topic_name))

                    # Search for comment
                    comment = self.search_comment(node.lineno)

                    self.stats[func_name].append(
                        {"topic": topic_name,
                         "lineno": node.lineno,
                         "type": self.parse_datatype(msg_type),
                         "comment": comment})

    def visit_Import(self, node):
        for alias in node.names:
            self.stats["import"].append(alias.name)
        self.generic_visit(node)

    def visit_ImportFrom(self, node):
        if isinstance(node, ast.ImportFrom):
            for alias in node.names:
                asname = alias.name
                if alias.asname:
                    asname = alias.asname
                # print "Imported ID {}: {}/{}".format(asname, node.module, alias.name)

                self.from_imports[asname] = "{}/{}".format(node.module, alias.name)

        self.generic_visit(node)

    # def visit_Assign(self, node):
        # for target in node.targets:
        # if isinstance(target, ast.Name):
        # print "{}: {}".format(target.id, node.value)
        # elif isinstance(target, ast.Attribute):
        # print "{}.{}: {}".format(target.value.id, target.attr, node.value)
        # self.generic_visit(node)


class PythonParser(object):
    """Parser for python nodes which fills the node representation"""

    def __init__(self, filename):
        node_name = filename.split('/')[-1].strip(".py")
        self.node = Node(node_name)
        self.filename = filename.split('/')[-1]
        #                    regex        as_type    add_function
        self.parser_fcts = [("get_param", Parameter, self.node.add_parameter),
                            ("Subscriber", Subscriber, self.node.add_subscriber),
                            ("Publisher", Publisher, self.node.add_publisher),
                            ("SimpleActionClient", ActionClient, self.node.add_action_client),
                            ("ServiceProxy", ServiceClient, self.node.add_service_client),
                            ("Service", Service, self.node.add_service),
                            ("SimpleActionServer", Action, self.node.add_action)]
        with open(filename) as filecontent:
            self.lines = filecontent.readlines()
        with open(filename, "r") as source:
            self.tree = ast.parse(source.read())
        self.tokens = list()
        with open(filename, "rb") as source:
            for five_tuple in tokenize.generate_tokens(source.readline):
                self.tokens.append(five_tuple)

        self.parse()

    def parse(self):
        """
        Parses the lines extracted from file in init method.
        Therefore extract and add all relevant features from python node including comments on them.
        """
        analyzer = Analyzer(self.tokens)
        analyzer.visit(self.tree)

        for rospy_fcn, astype, add_fcn in self.parser_fcts:
            for item in analyzer.stats[rospy_fcn]:
                if rospy_fcn == "get_param":
                    # print (item)
                    new_item = astype(name=item["name"],
                                      description=item["comment"],
                                      default_value=item["default"]
                                      )
                else:
                    new_item = astype(name=item["topic"],
                                      description=item["comment"],
                                      datatype=item["type"])
                new_item.filename = self.filename
                new_item.line_number = item["lineno"]
                new_item.code = self.lines[new_item.line_number - 1]
                add_fcn(new_item)
