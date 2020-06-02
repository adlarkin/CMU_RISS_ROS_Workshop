# Using Custom Message Types

_Previous tutorial: [Writing Our Own Publisher and Subscriber](./5_publisher_subscriber.md)_

So far, we have created a ROS publisher that sends two integers to a topic, and a ROS subscriber that adds the integers published to this topic.
We used the `std_msgs/Int64MultiArray` message type to hold the two integers.

If you take a closer look at the `std_msgs/Int64MultiArray` message [definition](http://docs.ros.org/api/std_msgs/html/msg/Int64MultiArray.html), you'll notice that this message type is a bit overkill for our use case.
This message type can handle multi-dimensional arrays, but we don't need that.
All we need is a way to hold two integers, which we accomplished originally by making a one-dimensional array that has two elements in it.

It's common that the message types in the [std_msgs](https://wiki.ros.org/std_msgs) package are not sufficient for your application's use case.
This means that you need to create your own message type.
In this tutorial, we will go over how to create ROS messages, creating our own message type that contains two integers.

_Most of the steps below are based on the [official ROS msg tutorial](https://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)._

## Creating the Message

Let's create our new message as a separate package - we'll call this message package `two_ints_msg`.
It's good practice to make messages as a separate package so that they can be easiy re-used by other packages.
The new message we are creating will consist of two integers.
We will be using the [std_msgs/Int64](http://docs.ros.org/api/std_msgs/html/msg/Int64.html) message type to help us represent an integer, so our new package will depend on `std_msgs`:

```
$ cd ~/ws/src/
$ catkin_create_pkg two_ints_msg std_msgs
```

We didn't add a dependency to `roscpp` this time because we don't need to write any c++ code when creating a ROS message.

Let's make sure we update the `description` and `license` tags in the `package.xml` file:

```xml
<description>Defines a message type of 2 64-bit integers.</description>

<license>MIT</license>
```

Now, we can define our two-integer message.
Messages are defined via `.msg` files, which are placed in a `msg` dir inside of the package:

```
$ cd two_ints_msg
$ mkdir msg
$ touch msg/TwoInts.msg
```

The file structure for the `two_ints_msg` package should now look like this:

```
two_ints_msg
├── CMakeLists.txt
├── msg
│   └── TwoInts.msg
└── package.xml

1 directory, 3 files
```

Let's go ahead and place our message definition in the `TwoInts.msg` file.
Copy and paste the following into this file:

```
int64 first
int64 second
```

We just defined our message as two 64-bit integers named _first_ and _second_.
Before we build this package, we need to add the following lines to the package's `package.xml`:

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

These dependencies will turn the content of the `.msg` file into source code for c++ and python.
Your `package.xml` should now look like this:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>two_ints_msg</name>
  <version>0.0.0</version>
  <description>Defines a message type of 2 64-bit integers.</description>

  <maintainer email="dev@todo.todo">dev</maintainer>

  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>std_msgs</exec_depend>
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
</package>
```

We also need to modify the package's `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(two_ints_msg)

## use C++11
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  TwoInts.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## Tell other catkin packages that they need the message_runtime package
## in order to use the message defined in this package
catkin_package(
  CATKIN_DEPENDS message_runtime
)
```

That's it!
Let's make sure that our message can be built:

```
cd ~/ws/

# we only need to build the message package to test it, not the adder package
$ colcon build --packages-select two_ints_msg
```

Now that we have built our message package, let's test it with `rosmsg`.
Run the following commands in a new shell:

```
$ source ~/ws/install/setup.bash
$ rosmsg show two_ints_msg/TwoInts
```

We should see output that matches the definition we used in our `.msg` file:

```
int64 first
int64 second
```

## Using the Message

Let's use this newly created message in the `adder` package.
We will create another publisher and subscriber that mimic the publisher and subscriber we wrote in the previous tutorial.
The new publisher and subscriber will use our `two_ints_msg/TwoInts` message type instead of the `std_msgs/Int64MultiArray` message type.

In the `adder/src/` directory, create a file named `two_ints_msg_publisher.cpp` and place the following code in it:

```c++
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
```

You'll notice that this file is not very different than `std_msgs_publisher.cpp`.
Take a moment to compare these two publisher files, noting how different message types are used.

Next, create a file named `two_ints_msg_subscriber.cpp` in `adder/src` and place the following code in it:

```c++
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
```

Again, this file mimics `std_msgs_publisher.cpp`, but uses a different message type.

We need to update `adder`'s `CMakeLists.txt` to generate these new publisher and subscruber executables:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(adder)

## use C++11
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  two_ints_msg
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_std_msgs_pub src/std_msgs_publisher.cpp src/rand_int_generator.cpp)
add_executable(${PROJECT_NAME}_std_msgs_sub src/std_msgs_subscriber.cpp)
add_executable(${PROJECT_NAME}_two_ints_msg_pub src/two_ints_msg_publisher.cpp src/rand_int_generator.cpp)
add_executable(${PROJECT_NAME}_two_ints_msg_sub src/two_ints_msg_subscriber.cpp)

target_link_libraries(${PROJECT_NAME}_std_msgs_pub
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_std_msgs_sub
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_two_ints_msg_pub
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_two_ints_msg_sub
  ${catkin_LIBRARIES}
)

install(
  TARGETS
  ${PROJECT_NAME}_std_msgs_pub
  ${PROJECT_NAME}_std_msgs_sub
  ${PROJECT_NAME}_two_ints_msg_pub
  ${PROJECT_NAME}_two_ints_msg_sub
  RUNTIME DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
```

The contents of this `CMakeLists.txt` file should be familiar to you by now, but it's important to note that we also had to add the `two_ints_msg` package to our `find_package(catkin REQUIRED COMPONENTS ...)` call.

Let's make sure we don't forget to update the `adder`'s `package.xml` file as well so that the dependency on the `two_ints_msg` is properly documented:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>adder</name>
  <version>0.0.0</version>
  <description>Adds two ints together via a publisher/subscriber.</description>

  <maintainer email="dev@todo.todo">dev</maintainer>

  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <depend>std_msgs</depend>
  <depend>two_ints_msg</depend>
</package>
```

Great! The `adder` package has now been set up to use the message we created in the `two_ints_msg` package.

### Testing the Custom Message Publisher and Subscriber

Let's run our new publisher and subscriber to make sure that they work:

```
# in one shell, start the ROS master
$ roscore

# in another shell, start the publisher
$ source ~/ws/install/setup.bash
$ rosrun adder adder_two_ints_msg_pub

# in another shell, start the subscriber
$ source ~/ws/install/setup.bash
$ rosrun adder adder_two_ints_msg_sub
```

You should see two randomly generated integers being sent over the ROS network and added together.

# Next Tutorial

[Roslaunch](./7_roslaunch.md)