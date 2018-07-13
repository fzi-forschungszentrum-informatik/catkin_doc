#!/usr/bin/env python

import unittest
import catkin_doc.python
import os.path

class TestPython(unittest.TestCase):
    """Test basic functionality of the python doc module"""
    def test_extract_params(self):
        node = catkin_doc.python.PythonParser("setup.py")
        self.assertTrue(
            node.extract_param(
                "self.stop_robot_topic = rospy.get_param('~stop_robot_topic', '/hello')"))
        self.assertTrue(
            node.extract_param(
                "self.stop_robot_topic = rospy.get_param('~stop_robot_topic')"))
        self.assertTrue(
            node.extract_param(
                'global_name = rospy.get_param("/global_name")'))

    def test_extract_params_false(self):
        node = catkin_doc.python.PythonParser("setup.py")
        self.assertFalse(
            node.extract_param( "rospy.loginfo('~stop_robot_topic')")[1])

    def test_extract_subs(self):
        parser = catkin_doc.python.PythonParser("setup.py")
        self.assertTrue(
            parser.extract_sub(
                'rospy.Subscriber("chatter", String, callback)'))
        self.assertTrue(
            parser.extract_sub(
                'self.joint_state_sub = rospy.Subscriber("pathloader/reordered_joint_states", JointState, self.joint_status_changed)'))

    def test_extract_subs_false(self):
        parser = catkin_doc.python.PythonParser("setup.py")
        self.assertFalse(
            parser.extract_sub( "rospy.loginfo('~stop_robot_topic')")[1])

    def test_extract_pubs(self):
        parser = catkin_doc.python.PythonParser("setup.py")
        self.assertTrue(
            parser.extract_pub(
                "pub = rospy.Publisher('chatter', String, queue_size=10)"))
        self.assertTrue(
            parser.extract_pub(
                "pub = rospy.Publisher('chatter', String, queue_size=10, latch=True)"))

    def test_extract_pubs_false(self):
        parser = catkin_doc.python.PythonParser("setup.py")
        self.assertFalse(
            parser.extract_pub( "rospy.loginfo('~stop_robot_topic')")[1])

    def test_action_clients(self):
        parser = catkin_doc.python.PythonParser("setup.py")
        self.assertTrue(
            parser.extract_action_client(
                "self.action_client = actionlib.SimpleActionClient('pathloader', PlayTrajectoryAction)"))

    def test_action_clients_false(self):
        parser = catkin_doc.python.PythonParser("setup.py")
        self.assertFalse(
            parser.extract_action_client( "rospy.loginfo('~stop_robot_topic')")[1])

    def test_service_clients(self):
        parser = catkin_doc.python.PythonParser("setup.py")
        self.assertTrue(
            parser.extract_service_client(
                "append_points = rospy.ServiceProxy('pathloader/appendPoints', ChangePath)"))

    def test_service_clients_false(self):
        parser = catkin_doc.python.PythonParser("setup.py")
        self.assertFalse(
            parser.extract_service_client( "rospy.loginfo('~stop_robot_topic')")[1])

    def test_service(self):
        parser = catkin_doc.python.PythonParser("setup.py")
        self.assertTrue(
            parser.extract_service(
                "s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)"))

    def test_service_false(self):
        parser = catkin_doc.python.PythonParser("setup.py")
        self.assertFalse(
            parser.extract_service( "rospy.loginfo('~stop_robot_topic')")[1])

    def test_action(self):
        parser = catkin_doc.python.PythonParser("setup.py")
        self.assertTrue(
            parser.extract_action(
                "self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start=False)"))

    def test_action_false(self):
        parser = catkin_doc.python.PythonParser("setup.py")
        self.assertFalse(
            parser.extract_action( "rospy.loginfo('~stop_robot_topic')")[1])

    def test_comment(self):
        parser = catkin_doc.python.PythonParser("setup.py")
        self.assertTrue(
            parser.extract_comment(
                "#This should be recognized as comment") == "This should be recognized as comment")

    def test_file_exist(self):
        parser = catkin_doc.python.PythonParser("setup.py")
        succ, param , value = parser.extract_param(
          "self.stop_robot_topic = rospy.get_param('~stop_robot_topic', '/hello')")
        parser.add_param(param, value, "Some kind of comment")
        succ, param, no_value = parser.extract_param(
          "self.stop_robot_topic = rospy.get_param('~start_robot_topic')")
        parser.add_param(param, no_value, "")
        succ, sub , value = parser.extract_sub(
            'self.joint_state_sub = rospy.Subscriber("pathloader/reordered_joint_states", JointState, self.joint_status_changed)')
        parser.add_sub(sub, value, "")
        succ, pub , value = parser.extract_pub(
            "pub = rospy.Publisher('chatter', String, queue_size=10)")
        parser.add_pub(pub, value, "")
        succ, ac , value = parser.extract_action_client(
            "self.action_client = actionlib.SimpleActionClient('pathloader', PlayTrajectoryAction)")
        parser.add_action_client(ac, value, "")
        succ, sc, value = parser.extract_service_client(
            "append_points = rospy.ServiceProxy('pathloader/appendPoints', ChangePath)")
        parser.add_service_client(sc, value, "")
        succ, srv , value = parser.extract_service(
            "s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)")
        parser.add_service(srv, value, "")
        succ, act, value = parser.extract_action(
            "self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start=False)")
        parser.add_action(act, value, "")
        parser.node.node_to_md_file()
        self.assertTrue(
          os.path.isfile("README.md"))
