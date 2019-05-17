#!/usr/bin/env python

import unittest
import catkin_doc.parsers.cppparser
from catkin_doc.parsers.cppparser import CppParser
import os.path

from catkin_doc.datastructures.parameter import Parameter
from catkin_doc.datastructures.service import Service, ServiceClient
from catkin_doc.datastructures.action import Action, ActionClient
from catkin_doc.datastructures.topic import Subscriber, Publisher


class TestCpp(unittest.TestCase):
    """Test basic functionality of the cpp doc module"""

    def test_extract_params(self):
        #Just add setup.py as file to parse as a file is needed to instance the parser but it is not really used here
        node = CppParser("test", ["setup.py"])
        self.assertTrue(
            node.extract_info(
                'ros::param::param<std::string>("param1", param1, "default_value1");', Parameter, node.param_regex)[0])
        self.assertTrue(
            node.extract_info(
                'ros::param::get("/param2", param2)', Parameter, node.param_regex_alt2)[0])
        self.assertTrue(
            node.extract_info(
                'nh.getParam("param3", param3)', Parameter, node.param_regex_alt1)[0])
        self.assertTrue(
            node.extract_info(
                'nh.param<std::string>("param4", param4, "default_value4");', Parameter, node.param_regex)[0])

    def test_extract_params_false(self):
        node = CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_info(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);', Parameter, node.param_regex)[0])

    def test_subscriber(self):
        node = CppParser("test", ["setup.py"])
        self.assertTrue(
            node.extract_info(
                ' m_start_stop_sub = m_glob_node_handle.subscribe("start_stop", 1, &PathLoader::startStopCallback, this);', Subscriber, node.subscriber_regex)[0])
        self.assertTrue(
            node.extract_info(
                'ros::Subscriber sub = nh.subscribe<std_msgs::String>("my_topic", 1, Foo());', Subscriber, node.subscriber_regex)[0])

    def test_subscriber_false(self):
        node = CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_info(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);', Subscriber, node.subscriber_regex)[0])

    def test_publisher(self):
        node = CppParser("test", ["setup.py"])
        self.assertTrue(
            node.extract_info(
                'ros::Publisher pub = nh.advertise<std_msgs::String>("topic_name", 5);', Publisher, node.publisher_regex)[0])

    def test_publisher_false(self):
        node = CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_info(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);', Publisher, node.publisher_regex)[0])

    def test_service(self):
        node = CppParser("test", ["setup.py"])
        self.assertTrue(
            node.extract_info(
                'ros::ServiceServer srv = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("my_service", Foo());', Service, node.service_regex)[0])
        self.assertTrue(
            node.extract_info(
                'ros::ServiceServer service = nh.advertiseService("my_service", &Foo::callback, &foo_object);', Service, node.service_regex)[0])

    def test_service_false(self):
        node = CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_info(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);', Service, node.service_regex)[0])

    def test_service_clients(self):
        node = CppParser("test", ["setup.py"])
        self.assertTrue(
            node.extract_info(
                'ros::service::call("my_service_name", foo)', ServiceClient, node.service_client_regex_alt)[0])
        self.assertTrue(
            node.extract_info(
                'ros::ServiceClient client = nh.serviceClient<my_package::Foo>("my_service_name");', ServiceClient, node.service_client_regex)[0])

    def test_service_clients_false(self):
        node = CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_info(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);', ServiceClient, node.service_client_regex)[0])

    def test_action_client(self):
        node = CppParser("test", ["setup.py"])
        self.assertTrue(
            node.extract_info(
                ' m_ros_control_action_client = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("controller_topic", true);', ActionClient, node.action_client_regex)[0])

    def test_action_client_false(self):
        node = CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_info(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);', ActionClient, node.action_client_regex)[0])

    def test_action_server(self):
        node = CppParser("test", ["setup.py"])
        self.assertTrue(
            node.extract_info(
                'm_action_server = new actionlib::SimpleActionServer<fzi_manipulation_msgs::PlayTrajectoryAction>(m_private_nh, action_name, false);', Action, node.action_regex)[0])

    def test_action_server_false(self):
        node = CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_info(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);', Action, node.action_regex)[0])

    def test_comment(self):
         node = CppParser("test", ["setup.py"])
         self.assertTrue(
            catkin_doc.parsers.cppparser.extract_comment(
                "//This should be recognized as comment") == "This should be recognized as comment")

