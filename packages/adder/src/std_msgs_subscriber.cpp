#include <ros/ros.h>                  // ROS c++ client library
#include <std_msgs/Int64MultiArray.h> // std_msgs::Int64MultiArray message type

// A callback that is triggered whenever our subscriber receives a new message.
// Once this method is triggered, this method extracts the two integers from
// the received message and adds them together.
void SubscriberCB(const std_msgs::Int64MultiArrayConstPtr & msg);

int main(int argc, char *argv[])
{
  // Initialize a node called "std_msgs_sub" and register it with the ROS master
  ros::init(argc, argv, "std_msgs_sub");

  // Get a reference to this newly created node
  ros::NodeHandle nh;

  // Create a ROS subscriber.
  // This subscriber will listen to the "std_msgs_topic".
  // The second argument (10) is the queue size of the subscriber.
  // The last argument is the address of the callback method that should be triggered
  // whenever the subscriber receives a new message.
  ros::Subscriber sub = nh.subscribe("std_msgs_topic", 10, &SubscriberCB);

  // Enter an infinite loop, triggering callbacks whenever data is available.
  // This loop can be stopped by killing the node with something like ctrl-c.
  // Callbacks will never be triggered if we forget to call ros::spin()!
  ros::spin();
}

void SubscriberCB(const std_msgs::Int64MultiArrayConstPtr & msg)
{
  ROS_INFO_STREAM("Received numbers " << msg->data[0] << " & " << msg->data[1]);
  ROS_INFO_STREAM("Sum is " << msg->data[0] + msg->data[1] << "\n");
}