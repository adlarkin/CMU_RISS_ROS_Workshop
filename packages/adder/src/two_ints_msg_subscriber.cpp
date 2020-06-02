#include <ros/ros.h>
#include <two_ints_msg/TwoInts.h>

void SubscriberCB(const two_ints_msg::TwoIntsConstPtr & msg);

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "two_ints_msg_sub");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("two_ints_msg_topic", 10, &SubscriberCB);

  ros::spin();
}

void SubscriberCB(const two_ints_msg::TwoIntsConstPtr & msg)
{
  ROS_INFO_STREAM("Received numbers " << msg->first << " & " << msg->second);
  ROS_INFO_STREAM("Sum is " << msg->first + msg->second << "\n");
}