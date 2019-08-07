#include <ros/ros.h>
#include <std_msgs/String.h>

#include <test_package/listener.h>

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM(msg);
}

void chatterCallback2(const std_msgs::StringConstPtr& msg);

void callback1(boost::shared_ptr<std_msgs::String const> msg) {}
void callback2(boost::shared_ptr<std_msgs::String> msg) {}
void callback3(const boost::shared_ptr<std_msgs::String>& msg) {}
void callback4(const ros::MessageEvent<std_msgs::String const>& msg) {}
void callback5(const ros::MessageEvent<std_msgs::String>& msg) {}
void callback6(const std_msgs::String& msg) {}
void callback7(const std_msgs::String::Ptr& msg) {}
void callback8(const std_msgs::StringPtr& msg) {}
void callback9(std_msgs::String) {}
void callback10(std_msgs::String::ConstPtr msg) {}
void callback11(std_msgs::String::Ptr msg) {}
void callback12(std_msgs::StringConstPtr msg) {}
void callback13(std_msgs::StringPtr msg) {}

int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "test_package_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  Listener my_listener;

  // The publisher
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  // The example subscriber.
  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

  // Functor subscriber
  ros::Subscriber functor_sub = nh.subscribe<std_msgs::String>("my_topic", 1, Listener());

  nh.subscribe("chatter", 1, callback1);
  nh.subscribe("chatter", 1, callback2);
  nh.subscribe("chatter", 1, callback3);
  nh.subscribe("chatter", 1, callback4);
  nh.subscribe("chatter", 1, callback5);
  nh.subscribe("chatter", 1, callback6);
  nh.subscribe("chatter", 1, callback7);
  nh.subscribe("chatter", 1, callback8);
  nh.subscribe("chatter", 1, callback9);
  nh.subscribe("chatter", 1, callback10);
  nh.subscribe("chatter", 1, callback11);
  nh.subscribe("chatter", 1, callback12);
  nh.subscribe("chatter", 1, callback13);

  ros::spin();
  return 0;
}

void chatterCallback2(const std_msgs::StringConstPtr& msg)
{
  ROS_INFO_STREAM(*msg);
}
