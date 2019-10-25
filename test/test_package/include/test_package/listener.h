#ifndef TEST_PACKAGE_LISTENER_H_INCLUDED
#define TEST_PACKAGE_LISTENER_H_INCLUDED

#include <ros/ros.h>

#include <std_msgs/String.h>

class Listener
{
public:
  Listener();
  virtual ~Listener() = default;

  void operator()(const std_msgs::StringConstPtr& msg) {}

private:
  ros::NodeHandle nh_;
  ros::Subscriber function_sub_;
  ros::Subscriber event_sub_;

  void valueCallback(std_msgs::String msg);
  void messageEventCallback(const ros::MessageEvent<std_msgs::String const>& event);
};
#endif // ifndef TEST_PACKAGE_LISTENER_H_INCLUDED
