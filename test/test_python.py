#!/usr/bin/env python

import unittest
import catkin_doc.python

class TestPython(unittest.TestCase):
    """Test basic functionality of the python doc module"""
    def test_extract(self):
        node = catkin_doc.python.PythonNode()
        self.assertTrue(
            node.extract_params(
                "self.stop_robot_topic = rospy.get_param('~stop_robot_topic', '/hello')"))
        self.assertTrue(
            node.extract_params(
                "self.stop_robot_topic = rospy.get_param('~stop_robot_topic')"))

    def test_extract_false(self):
        node = catkin_doc.python.PythonNode()
        self.assertFalse(
            node.extract_params( "rospy.loginfo('~stop_robot_topic')"))
