#include <ros/ros.h>                  // ROS c++ client library
#include <std_msgs/Int64MultiArray.h> // std_msgs::Int64MultiArray message type
#include <adder/rand_int_generator.h> // Our re-usable random integer generator function

// Generates a std_msgs::Int64MultiArray message type,
// filling the data array of this message with 2 random integers between 0-10
std_msgs::Int64MultiArray generate_msg();

int main(int argc, char *argv[])
{
  // Initialize a node called "std_msgs_pub" and register it with the ROS master
  ros::init(argc, argv, "std_msgs_pub");

  // Get a reference to this newly created node
  ros::NodeHandle nh;

  // Create a ROS publisher.
  // By calling "advertise", we are telling the ROS master that there is a topic called
  // "std_msgs_topic" that works with std_msgs::Int64MultiArray messages.
  // The last argument (10) is the queue size of the publisher.
  ros::Publisher pub = nh.advertise<std_msgs::Int64MultiArray>("std_msgs_topic", 10);

  // Publish messages at a rate of (approximately) 1 Hz
  ros::Rate r(1);   // 1 Hz
  while (ros::ok()) // While ROS and this node are still running...
  {
    ROS_INFO("Publishing message...");
    auto msg = generate_msg();
    pub.publish(msg); // The publisher object performs the publishing here
    ROS_INFO("Message has been published\n");

    r.sleep();
  }
}

std_msgs::Int64MultiArray generate_msg()
{
  std_msgs::Int64MultiArray msg;
  msg.data.push_back(make_rand_int(0, 10));
  msg.data.push_back(make_rand_int(0, 10));

  return msg;
}