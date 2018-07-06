#!/usr/bin/env python

import unittest
import catkin_doc.python
import os.path

class TestPython(unittest.TestCase):
    """Test basic functionality of the python doc module"""
    def test_extract(self):
        node = catkin_doc.python.PythonParser()
        self.assertTrue(
            node.extract_params(
                "self.stop_robot_topic = rospy.get_param('~stop_robot_topic', '/hello')"))
        self.assertTrue(
            node.extract_params(
                "self.stop_robot_topic = rospy.get_param('~stop_robot_topic')"))
        self.assertTrue(
            node.extract_params(
                'global_name = rospy.get_param("/global_name")'))

    def test_extract_false(self):
        node = catkin_doc.python.PythonParser()
        self.assertFalse(
            node.extract_params( "rospy.loginfo('~stop_robot_topic')"))

    def test_file_exist(self):
        parser = catkin_doc.python.PythonParser()
        parser.extract_params(
          "self.stop_robot_topic = rospy.get_param('~stop_robot_topic', '/hello')")
        parser.extract_params(
          "self.stop_robot_topic = rospy.get_param('~start_robot_topic')")
        parser._node.node_to_md()
        self.assertTrue(
          os.path.isfile("README.md"))

