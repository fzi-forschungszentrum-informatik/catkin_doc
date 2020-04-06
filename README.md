[![Build badge](https://github.com/fzi-forschungszentrum-informatik/catkin_doc/workflows/Python%20package/badge.svg?branch=master)](https://github.com/fzi-forschungszentrum-informatik/catkin_doc/actions/)
[![codecov](https://codecov.io/gh/fzi-forschungszentrum-informatik/catkin_doc/branch/master/graph/badge.svg)](https://codecov.io/gh/fzi-forschungszentrum-informatik/catkin_doc)


# The catkin\_doc project

Generally this package is for automatically generating documentation for ROS nodes. Documentation is
generated for each python and cpp node found in a given package.

For this, it will extract all nodes, their parameters, subscribers, etc and create doc entries for
them. If a comment in the code above the line creating any object exists, this is used as an initial
docstring.

## Installation
It's easiest to use pip to install this on a per-user basis:
```bash
# clone this repository and call from inside the cloned directory
pip install --user -e .
```
As `pip --user` installs to your local space, you might have to add `~/.local/bin` to your path.

## How to use:

Please consider that the generated `README.md` is currently put in the
directory of the package being documented. If you already have a `README.md` not generated by this
package make sure to backup that before using this.

How to create new or update documentation for a package:
```
catkin_doc "/path/to/your/package"
```
If updating a previously generated documentation any description altered by hand will not be
automatically overwritten. If descriptions from code and a previously generated documentation
differ, the user can choose the correct version.

## How to have your docstrings extracted from code
`catkin_doc` encourages you to document your ROS-API elements in your source code already. For
example

```cpp
// Maximum distance between two points to be added to the same cluster
m_cluster_dist_euclid = m_priv_nh.param("cluster_dist_euclid", 0.13);
```

will result in a docstrig such as:

> * **"cluster_dist_euclid"** (default: 0.13)
>
>    Maximum distance between two points to be added to the same cluster

Add your comments just above the line creating the objects

```cpp
// Param variant 1
m_value = nh.param("param_name", 0.13);
// Param variant 2
nh.param("param_name", m_value, 0.13);
// Param variant 3
nh.getParam("param_name", m_value);
// Param variant 4
ros::param::get("param_name", m_value)

// Subscriber
sub = nh.subscribe("topic", 10, msgCallback);
// Publisher
pub = nh.advertise<std_msgs::Bool>("has_info", 20);
```

It works the same way for types not being shown here or for python. If no description is given,
`catkin_doc` will instead create a placeholder description that will point to the respective line in
code:

> * **"~state"** (State)
>    
>    Please add description. See test_node.py line number: 14\
>    Code: Publisher('~state', State, queue_size=1, latch=True)

## Acknowledgment
Special thanks to **Lena Holoch**, who developed large parts of this project.
For a full list of contributors, please see [CONTRIBUTORS.md](CONTRIBUTORS.md)
