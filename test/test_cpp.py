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
