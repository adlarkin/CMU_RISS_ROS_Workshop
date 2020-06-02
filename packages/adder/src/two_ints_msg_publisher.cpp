#include <ros/ros.h>
#include <two_ints_msg/TwoInts.h>
#include <adder/rand_int_generator.h>

two_ints_msg::TwoInts generate_msg();

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "two_ints_msg_pub");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<two_ints_msg::TwoInts>("two_ints_msg_topic", 10);

  ros::Rate r(1);
  while (ros::ok())
  {
    ROS_INFO("Publishing message...");
    auto msg = generate_msg();
    pub.publish(msg);
    ROS_INFO("Message has been published\n");

    r.sleep();
  }
}

two_ints_msg::TwoInts generate_msg()
{
  two_ints_msg::TwoInts msg;
  msg.first = make_rand_int(0, 10);
  msg.second = make_rand_int(0, 10);

  return msg;
}