# test_package

The test_package package

## Launchfiles

### test_package_node_with_arg.launch

#### Arguments

##### name (default: "default")

Please add description. See file "launch/test_package_node_with_arg.launch".

### test_package_node.launch

No arguments for this launch file found. You can add a description by hand, if you like.

## Nodes

### test_package_node



#### Published topics

##### chatter ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

The publisher

#### Subscribed topics

##### chatter ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

The example subscriber.

##### chatter1 (boost/shared_ptr<std_msgs/String)

Please add description. See test_package_node.cpp line number: 49

	nh.subscribe("chatter1", 1, callback1);

##### chatter10 ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Please add description. See test_package_node.cpp line number: 58

	nh.subscribe("chatter10", 1, callback10);

##### chatter11 ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Please add description. See test_package_node.cpp line number: 59

	nh.subscribe("chatter11", 1, callback11);

##### chatter12 ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Please add description. See test_package_node.cpp line number: 60

	nh.subscribe("chatter12", 1, callback12);

##### chatter13 ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Please add description. See test_package_node.cpp line number: 61

	nh.subscribe("chatter13", 1, callback13);

##### chatter2 (boost/shared_ptr<std_msgs/String>)

Please add description. See test_package_node.cpp line number: 50

	nh.subscribe("chatter2", 1, callback2);

##### chatter3 (boost/shared_ptr<std_msgs/String>)

Please add description. See test_package_node.cpp line number: 51

	nh.subscribe("chatter3", 1, callback3);

##### chatter4 (ros/MessageEvent<std_msgs/String)

Please add description. See test_package_node.cpp line number: 52

	nh.subscribe("chatter4", 1, callback4);

##### chatter5 (ros/MessageEvent<std_msgs/String>)

Please add description. See test_package_node.cpp line number: 53

	nh.subscribe("chatter5", 1, callback5);

##### chatter6 ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Please add description. See test_package_node.cpp line number: 54

	nh.subscribe("chatter6", 1, callback6);

##### chatter7 ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Please add description. See test_package_node.cpp line number: 55

	nh.subscribe("chatter7", 1, callback7);

##### chatter8 ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Please add description. See test_package_node.cpp line number: 56

	nh.subscribe("chatter8", 1, callback8);

##### chatter9 ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Please add description. See test_package_node.cpp line number: 57

	nh.subscribe("chatter9", 1, callback9);

##### my_topic ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Functor subscriber

##### my_topic2 ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Please add description. See test_package_node.cpp line number: 45

	ros::Subscriber functor_sub2 = nh.subscribe<std_msgs::String>("my_topic2",
      1,
      Listener());

##### my_topic_event (None)

Please add description. See listener.cpp line number: 6

	event_sub_ = nh_.subscribe("my_topic_event", 1, &Listener::messageEventCallback, this);

##### my_topic_fun (None)

Please add description. See listener.cpp line number: 5

	function_sub_ = nh_.subscribe("my_topic_fun", 1, &Listener::valueCallback, this);

### python_node



#### Subscribed topics

##### topic1 ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))

Please add description. See python_node line number: 14

	        self.sub1 = rospy.Subscriber("topic1", Bool, self.sub_cb1)


##### topic2 ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))

This is a comment for sub2

##### topic3 ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))

Please add description. See python_node line number: 19

	        self.sub3 = rospy.Subscriber("topic3", Bool, self.sub_cb3, None)


