#!/usr/bin/env python
# coding=utf-8

from setuptools import setup
import sys
import os


def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()


setup(
    name="catkin_doc",
    description="Generate ROS API documentation of catkin package",
    long_description=read("README.md"),
    author="Felix Mauch",
    author_email="mauch@fzi.de",
    license="BSD",
    keywords=["catkin", "ROS"],
    packages=["catkin_doc"],
    package_dir={"": "src"},
    scripts=["bin/catkin_doc"],
    version=0.1,
    install_requires=["markdown-strings", "magic"],
    # test_suite    = "nose.collector",
    # classifiers   = [
    # "Development Status :: 5 - Production/Stable",
    # "Intended Audience :: Developers",
    # "License :: OSI Approved :: BSD License",
    # "Topic :: Software Development :: Quality Assurance",
    # "Environment :: Console",
    # "Operating System :: OS Independent",
    # "Programming Language :: Python",
    # "Programming Language :: Python :: 3"
    # ],
    # entry_points  = {
    # "catkin_tools.commands.catkin.verbs": [
    # "lint = catkin_lint.main:description",
    # ],
    # },
)
