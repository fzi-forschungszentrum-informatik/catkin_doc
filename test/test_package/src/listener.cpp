#include <test_package/listener.h>

Listener::Listener()
{
  function_sub_ = nh_.subscribe("my_topic_fun", 1, &Listener::valueCallback, this);
  event_sub_ = nh_.subscribe("my_topic_event", 1, &Listener::messageEventCallback, this);
}

void Listener::valueCallback(std_msgs::String msg) {}

void Listener::messageEventCallback(const ros::MessageEvent<std_msgs::String const>& event)
{
}
