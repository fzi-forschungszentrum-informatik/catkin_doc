#!/usr/bin/env python

import unittest
import catkin_doc.parsers.pythonparser
from catkin_doc.parsers.pythonparser import PythonParser
import os.path

from catkin_doc.datastructures.parameter import Parameter
from catkin_doc.datastructures.service import Service, ServiceClient
from catkin_doc.datastructures.action import Action, ActionClient
from catkin_doc.datastructures.topic import Subscriber, Publisher

class TestPython(unittest.TestCase):
    """Test basic functionality of the python doc module"""
    def test_extract_params(self):
        node = PythonParser("setup.py")
        self.assertTrue(
            catkin_doc.parsers.pythonparser.extract_info(
                "self.stop_robot_topic = rospy.get_param('~stop_robot_topic', '/hello')", Parameter, node.param_regex)[0])
        self.assertTrue(
            catkin_doc.parsers.pythonparser.extract_info(
                "self.stop_robot_topic = rospy.get_param('~stop_robot_topic')", Parameter, node.param_regex)[0])
        self.assertTrue(
            catkin_doc.parsers.pythonparser.extract_info(
                'global_name = rospy.get_param("/global_name")', Parameter, node.param_regex)[0])

    def test_extract_params_false(self):
        node = PythonParser("setup.py")
        self.assertFalse(
            catkin_doc.parsers.pythonparser.extract_info( "rospy.loginfo('~stop_robot_topic')", Parameter, node.param_regex)[0])

    def test_extract_subs(self):
        parser = PythonParser("setup.py")
        self.assertTrue(
            catkin_doc.parsers.pythonparser.extract_info(
                'rospy.Subscriber("chatter", String, callback)', Subscriber, parser.subscriber_regex)[0])
        self.assertTrue(
            catkin_doc.parsers.pythonparser.extract_info(
                'self.joint_state_sub = rospy.Subscriber("pathloader/reordered_joint_states", JointState, self.joint_status_changed)', Subscriber, parser.subscriber_regex)[0])

    def test_extract_subs_false(self):
        parser = PythonParser("setup.py")
        self.assertFalse(
            catkin_doc.parsers.pythonparser.extract_info( "rospy.loginfo('~stop_robot_topic')", Subscriber, parser.subscriber_regex)[0])

    def test_extract_pubs(self):
        parser = PythonParser("setup.py")
        self.assertTrue(
            catkin_doc.parsers.pythonparser.extract_info(
                "pub = rospy.Publisher('chatter', String, queue_size=10)", Publisher, parser.publisher_regex)[0])
        self.assertTrue(
            catkin_doc.parsers.pythonparser.extract_info(
                "pub = rospy.Publisher('chatter', String, queue_size=10, latch=True)", Publisher, parser.publisher_regex)[0])

    def test_extract_pubs_false(self):
        parser = PythonParser("setup.py")
        self.assertFalse(
            catkin_doc.parsers.pythonparser.extract_info( "rospy.loginfo('~stop_robot_topic')", Publisher, parser.publisher_regex)[0])

    def test_action_clients(self):
        parser = PythonParser("setup.py")
        self.assertTrue(
            catkin_doc.parsers.pythonparser.extract_info(
                "self.action_client = actionlib.SimpleActionClient('pathloader', PlayTrajectoryAction)", ActionClient, parser.action_client_regex)[0])

    def test_action_clients_false(self):
        parser = PythonParser("setup.py")
        self.assertFalse(
            catkin_doc.parsers.pythonparser.extract_info( "rospy.loginfo('~stop_robot_topic')", ActionClient, parser.action_client_regex)[0])

    def test_service_clients(self):
        parser = PythonParser("setup.py")
        self.assertTrue(
            catkin_doc.parsers.pythonparser.extract_info(
                "append_points = rospy.ServiceProxy('pathloader/appendPoints', ChangePath)", ServiceClient, parser.service_client_regex)[0])

    def test_service_clients_false(self):
        parser = PythonParser("setup.py")
        self.assertFalse(
            catkin_doc.parsers.pythonparser.extract_info( "rospy.loginfo('~stop_robot_topic')", ServiceClient, parser.service_client_regex)[0])

    def test_service(self):
        parser = PythonParser("setup.py")
        self.assertTrue(
            catkin_doc.parsers.pythonparser.extract_info(
                "s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)", Service, parser.service_regex)[0])

    def test_service_false(self):
        parser = PythonParser("setup.py")
        self.assertFalse(
            catkin_doc.parsers.pythonparser.extract_info( "rospy.loginfo('~stop_robot_topic')", Service, parser.service_regex)[0])

    def test_action(self):
        parser = PythonParser("setup.py")
        self.assertTrue(
            catkin_doc.parsers.pythonparser.extract_info(
                "self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start=False)", Action, parser.action_regex)[0])

    def test_action_false(self):
        parser = PythonParser("setup.py")
        self.assertFalse(
            catkin_doc.parsers.pythonparser.extract_info( "rospy.loginfo('~stop_robot_topic')", Action, parser.action_regex)[0])

    def test_comment(self):
        self.assertTrue(
            catkin_doc.parsers.pythonparser.extract_comment(
                "#This should be recognized as comment") == "This should be recognized as comment")

