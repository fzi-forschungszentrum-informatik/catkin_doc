#!/usr/bin/env python
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


from __future__ import print_function

import unittest
import tempfile
import catkin_doc.parsers.cppparser as cppparser
from catkin_doc.parsers.cppparser import CppParser
import os.path

import catkin_doc.datastructures as ds
from catkin_doc.datastructures.parameter import Parameter
from catkin_doc.datastructures.service import Service, ServiceClient
from catkin_doc.datastructures.action import Action, ActionClient
from catkin_doc.datastructures.topic import Subscriber, Publisher


class TestCpp(unittest.TestCase):
    """Test basic functionality of the cpp doc module"""

    def test_extract_params(self):
        # Just add setup.py as file to parse as a file is needed to instance
        # the parser but it is not really used here
        node = CppParser("test", ["setup.py"])
        code = 'ros::param::param<std::string>("param1", param1, "default_value1");'
        param, brackets = node.extract_info(code, Parameter, node.param_regex)
        self.assertEqual(param.name, "param1")
        self.assertEqual(param.namespace, None)
        self.assertEqual(param.default_value, "default_value1")
        self.assertEqual(param.datatype, "std::string")
        self.assertEqual(brackets, code)

        code = 'ros::param::get("/param2", param2)'
        param, brackets = node.extract_info(code, Parameter, node.param_regex)
        self.assertEqual(param.name, "param2")
        self.assertEqual(param.namespace, "/")
        self.assertEqual(brackets, code)

        code = 'nh.getParam("param3", param3)'
        param, brackets = node.extract_info(code, Parameter, node.param_regex)
        self.assertEqual(param.name, "param3")
        self.assertEqual(param.namespace, None)
        self.assertEqual(brackets, code)
        self.assertEqual(param.default_value, None)

        code = 'nh.param<std::string>("param4", param4, defaultGenerator());'
        param, brackets = node.extract_info(code, Parameter, node.param_regex)
        self.assertEqual(param.name, "param4")
        self.assertEqual(param.namespace, None)
        self.assertEqual(param.default_value, "defaultGenerator()")
        self.assertEqual(param.datatype, "std::string")
        self.assertEqual(brackets, code)

        code = '''m_nh.param<std::string>("waypoint_directory",
                          waypoint_dir,
                          ros::package::getPath("waypoint_navigation") + "/etc/waypoints");
'''
        param, brackets = node.extract_info(code, Parameter, node.param_regex)
        self.assertEqual(param.name, "waypoint_directory")
        self.assertEqual(param.namespace, None)
        self.assertEqual(param.default_value, "ros::package::getPath(\"waypoint_navigation\") + \"/etc/waypoints\"")
        self.assertEqual(param.datatype, "std::string")
        self.assertEqual(brackets, code)

    def test_extract_params_false(self):
        node = CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_info(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);',
                Parameter,
                node.param_regex)[0])

    def test_subscriber(self):
        node = CppParser("test", ["setup.py"])
        code = 'm_start_stop_sub=m_glob_node_handle.subscribe("start_stop", 1,'\
            '&PathLoader::startStopCallback, this) '

        sub, brackets = node.extract_info(code, Subscriber, node.subscriber_regex)

        self.assertEqual(sub.name, "start_stop")
        self.assertEqual(brackets, code)

        code = 'ros::Subscriber sub = nh.subscribe<std_msgs::String>("my_topic", 1, callback);'
        sub, brackets = node.extract_info(code, Subscriber, node.subscriber_regex)
        self.assertEqual(sub.name, "my_topic")
        self.assertEqual(sub.namespace, None)
        self.assertEqual(sub.datatype, "std_msgs/String")
        self.assertEqual(brackets, code)

    def test_subscriber_false(self):
        node = CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_info(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);', Subscriber, node.subscriber_regex)[0])

    def test_publisher(self):
        node = CppParser("test", ["setup.py"])
        code = 'ros::Publisher pub = nh.advertise<std_msgs::String>("topic_name", 5);'
        sub, brackets = node.extract_info(code, Publisher, node.publisher_regex)
        self.assertEqual(sub.name, "topic_name")
        self.assertEqual(sub.namespace, None)
        self.assertEqual(sub.datatype, "std_msgs/String")
        self.assertEqual(brackets, code)

        code = 'ros::Publisher pub = nh.advertise<std_msgs::String>("topic_name", 5, true);'
        sub, brackets = node.extract_info(code, Publisher, node.publisher_regex)
        self.assertEqual(sub.name, "topic_name")
        self.assertEqual(sub.namespace, None)
        self.assertEqual(sub.datatype, "std_msgs/String")
        self.assertEqual(brackets, code)

        code = 'nh.advertise<foo_msgs::Bar>(topic_name, 1,'\
            ' SubscriberConnectCallback,'\
            ' SubscriberDisconnectCallback,'\
            ' VoidConstPtr(),'\
            ' false)'
        sub, brackets = node.extract_info(code, Publisher, node.publisher_regex)
        self.assertEqual(sub.name, "topic_name")
        self.assertEqual(sub.namespace, None)
        self.assertEqual(sub.datatype, "foo_msgs/Bar")
        self.assertEqual(brackets, code)

    def test_publisher_false(self):
        node = CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_info(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);',
                Publisher,
                node.publisher_regex)[0])

    def test_service(self):
        node = CppParser("test", ["setup.py"])
        code = 'ros::ServiceServer srv = nh.advertiseService<std_srvs::Empty::Request, '\
            'std_srvs::Empty::Response>("my_service", Foo);'
        service, brackets = node.extract_info(code, Service, node.service_regex)
        self.assertEqual(service.name, "my_service")
        self.assertEqual(service.namespace, None)
        self.assertEqual(service.datatype, "std_srvs/Empty")
        self.assertEqual(brackets, code)

        code = 'ros::ServiceServer service = nh.advertiseService("my_service", '\
            '&Foo::callback, &foo_object);'
        service, brackets = node.extract_info(code, Service, node.service_regex)
        self.assertEqual(service.name, "my_service")
        self.assertEqual(brackets, code)

    def test_service_false(self):
        node = CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_info(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);', Service, node.service_regex)[0])

    def test_service_clients(self):
        node = CppParser("test", ["setup.py"])
        code = 'm_accept_path_client = '\
            'm_nh.serviceClient<follow_me_msgs::SetAdjustedPath>("/move_base/adjusted_plan");'
        service, brackets = node.extract_info(code, ServiceClient, node.service_client_regex)
        self.assertEqual(service.name, "adjusted_plan")
        self.assertEqual(service.namespace, "/move_base/")
        self.assertEqual(service.datatype, "follow_me_msgs/SetAdjustedPath")
        self.assertEqual(brackets, code)

        code = 'ros::service::call("my_service_name", std_srvs::Empty)'
        service, brackets = node.extract_info(code, ServiceClient, node.service_client_regex_alt)
        self.assertEqual(service.name, "my_service_name")
        self.assertEqual(service.namespace, None)
        self.assertEqual(service.datatype, "std_srvs/Empty")
        self.assertEqual(brackets, code)

    def test_service_clients_false(self):
        node = CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_info(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);', ServiceClient, node.service_client_regex)[0])

    def test_action_client(self):
        node = CppParser("test", ["setup.py"])
        code = 'm_ros_control_action_client = '\
            'new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>'\
            '("controller_topic", true);'
        action, brackets = node.extract_info(code, ActionClient, node.action_client_regex)
        self.assertEqual(action.name, "controller_topic")
        self.assertEqual(action.namespace, None)
        self.assertEqual(action.datatype, "control_msgs/FollowJointTrajectoryAction")
        self.assertEqual(brackets, code)

    def test_action_client_false(self):
        node = CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_info(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);', ActionClient, node.action_client_regex)[0])

    def test_action_server(self):
        node = CppParser("test", ["setup.py"])
        code = 'm_action_server = '\
            'new actionlib::SimpleActionServer<fzi_manipulation_msgs::PlayTrajectoryAction>'\
            '("execute_trajectory", boost::bind(&PathLoader::executeCB, this), false);'
        action, brackets = node.extract_info(code, Action, node.action_regex)
        self.assertEqual(action.name, "execute_trajectory")
        self.assertEqual(action.namespace, None)
        self.assertEqual(action.datatype, "fzi_manipulation_msgs/PlayTrajectoryAction")
        self.assertEqual(brackets, code)

    def test_action_server_false(self):
        node = CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_info(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);', Action, node.action_regex)[0])

    def test_comment(self):
        """Test extracting comments"""
        self.assertEqual(cppparser.extract_comment(
            "//This should be recognized as comment"), "This should be recognized as comment")
        self.assertEqual(cppparser.extract_comment(
            "// This should be recognized as comment"), "This should be recognized as comment")

        source_code = r'''//This should
// be recognized
   // as
      //comment
      node_object_here
'''
        source_file = tempfile.NamedTemporaryFile()
        source_file.write(source_code.encode(encoding="utf-8", errors="strict"))
        source_file.seek(0)
        parser = CppParser("comment", [source_file.name])
        self.assertEqual(parser.search_for_comment(4),
                         "This should be recognized as comment")

    def test_parsing(self):
        """Tests whole parsing of a file"""
        source_code = r'''// This is some initial comment
void chatterCallback(std_msgs::StringConstPtr& msg) {}
bool PathLoader::newTrajectory(fzi_manipulation_msgs::NewTrajectory::Request& req,
                               fzi_manipulation_msgs::NewTrajectory::Response& res) {}
ros::init();

// The publisher
chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

// The example subscriber. I know, the name's illegal
n.subscribe("chat;ter", 1000, chatterCallback);

ros::Subscriber sub2=n.subscribe<std_msgs::String> ("chatter",1, boost::bind(&stringcallback, _1, &test));

m_private_nh.advertiseService( "new_trajectory",
f_NewTrajectory(boost::bind(&PathLoader::newTrajectory, this, _1, _2)) );
'''
        source_file = tempfile.NamedTemporaryFile()
        source_file.write(source_code.encode(encoding="utf-8", errors="strict"))
        source_file.seek(0)
        print("Used source code:")
        data = source_file.readlines()
        for number, line in enumerate(data):
            print("{}: {}".format(number + 1, line))
        parser = CppParser("example_node", [source_file.name])

        node = parser.node

        self.assertEqual(node.name, "example_node")
        self.assertEqual(len(node.children), 3)
        self.assertEqual(node.children[ds.KEYS["publisher"]][0].name, "chatter")
        self.assertEqual(node.children[ds.KEYS["publisher"]][0].namespace, None)
        self.assertEqual(node.children[ds.KEYS["publisher"]][0].line_number, 8)
        self.assertEqual(node.children[ds.KEYS["publisher"]][0].datatype, "std_msgs/String")
        self.assertEqual(node.children[ds.KEYS["publisher"]][0].code,
                         data[7].decode("utf-8"))

        self.assertEqual(node.children[ds.KEYS["subscriber"]][0].name, "chat;ter")
        self.assertEqual(node.children[ds.KEYS["subscriber"]][0].namespace, None)
        self.assertEqual(node.children[ds.KEYS["subscriber"]][0].line_number, 11)
        self.assertEqual(node.children[ds.KEYS["subscriber"]][0].datatype, "std_msgs/String")
        self.assertEqual(node.children[ds.KEYS["subscriber"]]
                         [0].code, data[10].decode("utf-8"))
        self.assertEqual(node.children[ds.KEYS["subscriber"]]
                         [0].description, data[9].decode("utf-8").lstrip("// ").strip("\n"))

        self.assertEqual(node.children[ds.KEYS["subscriber"]][1].name, "chatter")
        self.assertEqual(node.children[ds.KEYS["subscriber"]][1].namespace, None)
        self.assertEqual(node.children[ds.KEYS["subscriber"]][1].line_number, 13)

        self.assertEqual(node.children[ds.KEYS["service"]][0].name, "new_trajectory")
        self.assertEqual(node.children[ds.KEYS["service"]][0].line_number, 15)
        self.assertEqual(node.children[ds.KEYS["service"]][0].namespace, None)
        self.assertEqual(node.children[ds.KEYS["service"]][0].datatype,
                         "fzi_manipulation_msgs/NewTrajectory")
