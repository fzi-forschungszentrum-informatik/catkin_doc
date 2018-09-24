#!/usr/bin/env python

import unittest
import catkin_doc.cpp
import os.path

class TestCpp(unittest.TestCase):
    """Test basic functionality of the cpp doc module"""

    def test_extract_params(self):
        #Just add setup.py as file to parse as a file is needed to instance the parser but it is not really used here
        node = catkin_doc.cpp.CppParser("test", ["setup.py"])
        self.assertTrue(
            node.extract_param(
                'ros::param::param<std::string>("param1", param1, "default_value1");')[0])
        self.assertTrue(
            node.extract_param(
                'ros::param::get("/param2", param2)')[0])
        self.assertTrue(
            node.extract_param(
                'nh.getParam("param3", param3)')[0])
        self.assertTrue(
            node.extract_param(
                'nh.param<std::string>("param4", param4, "default_value4");')[0])

    def test_extract_params_false(self):
        node = catkin_doc.cpp.CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_param(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);')[0])

    def test_subscriber(self):
        node = catkin_doc.cpp.CppParser("test", ["setup.py"])
        self.assertTrue(
            node.extract_sub(
                ' m_start_stop_sub = m_glob_node_handle.subscribe("start_stop", 1, &PathLoader::startStopCallback, this);')[0])
        self.assertTrue(
            node.extract_sub(
                'ros::Subscriber sub = nh.subscribe<std_msgs::String>("my_topic", 1, Foo());')[0])

    def test_subscriber_false(self):
        node = catkin_doc.cpp.CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_sub(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);')[0])

    def test_publisher(self):
        node = catkin_doc.cpp.CppParser("test", ["setup.py"])
        self.assertTrue(
            node.extract_pub(
                'ros::Publisher pub = nh.advertise<std_msgs::String>("topic_name", 5);')[0])

    def test_publisher_false(self):
        node = catkin_doc.cpp.CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_pub(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);')[0])

    def test_service(self):
        node = catkin_doc.cpp.CppParser("test", ["setup.py"])
        self.assertTrue(
            node.extract_service(
                'ros::ServiceServer srv = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("my_service", Foo());')[0])
        self.assertTrue(
            node.extract_service(
                'ros::ServiceServer service = nh.advertiseService("my_service", &Foo::callback, &foo_object);')[0])

    def test_service_false(self):
        node = catkin_doc.cpp.CppParser("test", ["setup.py"])
        self.assertFalse(
            node.extract_service(
                ' ros_control_action_service = m_nh.resolveName(ros_control_action_service);')[0])

    def test_file_exist(self):
        parser = catkin_doc.cpp.CppParser("test", ["setup.py"])
        succ, param , value = parser.extract_param(
          'ros::param::param<std::string>("param1", param1, "default_value1");')
        parser.add_param(param, value, "Some kind of comment")
        succ, sub , value = parser.extract_sub(
           'm_start_stop_sub = m_glob_node_handle.subscribe("start_stop", 1, &PathLoader::startStopCallback, this);')
        parser.add_sub(sub, value, "")
        parser.node.node_to_md_file()
        self.assertTrue(
          os.path.isfile("README.md"))
