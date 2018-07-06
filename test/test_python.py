#!/usr/bin/env python

import unittest
import catkin_doc.python
import os.path

class TestPython(unittest.TestCase):
    """Test basic functionality of the python doc module"""
    def test_extract_params(self):
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

    def test_extract_params_false(self):
        node = catkin_doc.python.PythonParser()
        self.assertFalse(
            node.extract_params( "rospy.loginfo('~stop_robot_topic')"))

    def test_extract_subs(self):
        parser = catkin_doc.python.PythonParser()
        self.assertTrue(
            parser.extract_subs(
                'rospy.Subscriber("chatter", String, callback)'))
        self.assertTrue(
            parser.extract_subs(
                'self.joint_state_sub = rospy.Subscriber("pathloader/reordered_joint_states", JointState, self.joint_status_changed)'))

    def test_extract_subs_false(self):
        parser = catkin_doc.python.PythonParser()
        self.assertFalse(
            parser.extract_subs( "rospy.loginfo('~stop_robot_topic')"))

    def test_extract_pubs(self):
        parser = catkin_doc.python.PythonParser()
        self.assertTrue(
            parser.extract_pubs(
                "pub = rospy.Publisher('chatter', String, queue_size=10)"))
        self.assertTrue(
            parser.extract_pubs(
                "pub = rospy.Publisher('chatter', String, queue_size=10, latch=True)"))

    def test_extract_pubs_false(self):
        parser = catkin_doc.python.PythonParser()
        self.assertFalse(
            parser.extract_pubs( "rospy.loginfo('~stop_robot_topic')"))

    def test_file_exist(self):
        parser = catkin_doc.python.PythonParser()
        parser.extract_params(
          "self.stop_robot_topic = rospy.get_param('~stop_robot_topic', '/hello')")
        parser.extract_params(
          "self.stop_robot_topic = rospy.get_param('~start_robot_topic')")
        parser.extract_subs(
            'self.joint_state_sub = rospy.Subscriber("pathloader/reordered_joint_states", JointState, self.joint_status_changed)')
        parser.extract_pubs(
            "pub = rospy.Publisher('chatter', String, queue_size=10)")
        parser.node.node_to_md()
        self.assertTrue(
          os.path.isfile("README.md"))
