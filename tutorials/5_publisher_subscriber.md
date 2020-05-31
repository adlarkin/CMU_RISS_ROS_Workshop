# Writing Our Own Publisher and Subscriber

_Previous tutorial: [Creating a ROS Package](./4_creating_ROS_package.md)_

Now that we have created the `adder` package, let's go over how to write ROS programs in c++.

As previously mentioned, this package will add two integers together.
We will need to do the following in order to do this via the ROS framework:

1. Create a node that publishes the integers to be added to a topic
2. Create a node that subscribes to this topic and adds together the integers received from this topic

We will go over how to write the publisher first, and then will discuss how to write the subscriber.

### Prep Work - Avoiding Code Duplication

The two integers that are being added together in this package are randomly generated.
We need to write a function that will randomly generate an integer `n`, such that `min_val <= n <= max_val`.
This function will be used in a few different files in the `adder` package as we continue to go through these tutorials,
so let's give this function its own header file so that we can re-use this header when needed.

Make a header file called `rand_int_generator.h` in `adder/include/adder/`.
Copy and paste the following code into this file:

```c++
#include <stdlib.h>

int make_rand_int(const int & min, const int & max);
```

Now that we have _declared_ this function in a `.h` file, we need a `.cpp` file that _defines_ this function.
Create a file called `rand_int_generator.cpp` in `adder/src/`.
Copy and paste the following code into this file:

```c++
#include <adder/rand_int_generator.h>

int make_rand_int(const int & min, const int & max)
{
  return rand() % (max + 1) + min;
}
```

## Writing a ROS Publisher

Let's go through how to configure files in the `adder` package for a ROS publisher.
This will consists of modifying the `CMakeLists.txt` and `package.xml` files, along with adding a new `.cpp` file that implements the publishing.

Modify your `CMakeLists.txt` to look like the following (I removed the auto-generated comments for readability):

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(adder)

## use C++11
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_std_msgs_pub src/std_msgs_publisher.cpp src/rand_int_generator.cpp)

target_link_libraries(${PROJECT_NAME}_std_msgs_pub
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_std_msgs_pub
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
```

If you are having a hard time understanding how this `CMakeLists.txt` file works, please take the time to either read through the auto-generated comments that were created by calling `catkin_create_pkg`, or read through the [catkin docs](https://docs.ros.org/api/catkin/html/howto/format2/index.html).

One thing I will point out is that we include `std_msgs` as a part of our `find_package(catkin REQUIRED COMPONENTS ...)` call.
This is because will be publishing the two integers through a [Int64MultiArray](http://docs.ros.org/api/std_msgs/html/msg/Int64MultiArray.html) message type, which is a part of the `std_msgs` package.

Modify your `package.xml` to look like the following (again, the auto-generated comments have been removed):

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
</package>
```

The only new line in this file is:

```xml
<depend>std_msgs</depend>
```

We needed to add this line to the `package.xml` so that the `catkin` build system (and others interested in using our package) know that this package also needs the `std_msgs` package in order to work.

Now that we have updated our `CMakeLists.txt` and `package.xml` files, let's create a new file called `std_msgs_publisher.cpp` in `adder/src/`.
Copy and paste the following code into this `.cpp` file:

```c++
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
```

Take some time to read through the comments in the code to gain a better understanding of how to use the ROS c++ client library.
If any of the code is difficult to understand (even after reading the comments), or if you'd like to learn more about writing a ROS publisher in c++, take a look at the [official ROS publisher/subscriber (c++) tutorial](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29).

### Testing our Publisher

Now that we have everything we need to publish messages to a topic, let's build our package and see how things work.

Let's build our `adder` package using `colcon`:

```
$ cd ~/ws
$ colcon build
```

If all goes well, you should see output similar to the following:

```
Starting >>> adder
Finished <<< adder [2.51s]

Summary: 1 package finished [2.63s]
```

Open another shell and start the ROS master if you don't have one running already:

```
$ roscore
```

Now, in another shell, let's try to run our publisher node with `rosrun`:

```
$ rosrun adder adder_std_msgs_pub
```

Unfortunately, we are given the following error:

```
[rospack] Error: package 'adder' not found
```

The reason for this is because although our package has been _built_, ROS doesn't know this.
So, in order to let ROS know about our new package, we need to run the following command (this is often called "sourcing a workspace"):

```
$ source ws/install/setup.bash
```

This command will add our package information (location on the file system, executables generated, etc) to an environment variable called `ROS_PACKAGE_PATH`, which is a list of paths that are searched for executables when running commands like `rosrun`:

```
# before running "source ws/install/setup.bash"
$ echo $ROS_PACKAGE_PATH/
/opt/ros/melodic/share/

# after running "source ws/install/setup.bash"
$ echo $ROS_PACKAGE_PATH/
/home/dev/ws/install/adder/share:/opt/ros/melodic/share/
```

So, as we can see, our `adder` package is now a part of the `ROS_PACKAGE_PATH`.
You'll need to source the workspace with the `adder` package every time a new shell is opened.

We never need to source ROS itself because the command for doing so (`source /opt/ros/melodic/setup.bash`) has actually been added to the container's `~/.bashrc` file (this was done while building the Docker image).
If you don't know much about shell environments, `~/.bashrc` is a file that sets up your shell environment every time a new shell is opened.

Now that ROS knows about our newly built package, let's try to run the `rosrun` command again:

```
$ rosrun adder adder_std_msgs_pub
```

You should now see some output from this node:

```
...

[ INFO] [1590885131.241582338]: Publishing message...
[ INFO] [1590885131.241705588]: Message has been published

[ INFO] [1590885132.241740550]: Publishing message...
[ INFO] [1590885132.241863166]: Message has been published

[ INFO] [1590885133.241887350]: Publishing message...
[ INFO] [1590885133.242026116]: Message has been published

...
```

Just to make sure that our topic has been created succesfully, let's use `rostopic`:

```
$ rostopic list
/rosout
/rosout_agg
/std_msgs_topic
```

As we can see, `/std_msgs_topic` exists!
We can even take a look at the data being published by our new node:

```
$ rostopic echo /std_msgs_topic
layout:
  dim: []
  data_offset: 0
data: [4, 7]
---
layout:
  dim: []
  data_offset: 0
data: [7, 8]
---
layout:
  dim: []
  data_offset: 0
data: [5, 1]

...
```

As we can see, our node is succesfully publishing 2 randomly generated integers between 0 and 10 to the `/std_msgs_topic`. Go ahead and kill the ROS master and our publisher node for now; we will run them again when we test our subscriber.

## Writing a ROS Subscriber

Now that we've gone over how ROS publishers work, let's discuss how to write a subcriber.
We won't need to change the `package.xml` this time (do you know why?), but we will need to add a few more things to the `CMakeLists.txt` file and create another `.cpp` file that implements the subscribing.

Your `CMakeLists.txt` should now look like this:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(adder)

## use C++11
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
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

target_link_libraries(${PROJECT_NAME}_std_msgs_pub
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_std_msgs_sub
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_std_msgs_pub ${PROJECT_NAME}_std_msgs_sub
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
```

There were 3 additions/changes made to our `CMakeLists.txt`:

1. Adding the executable for our subscriber:

```cmake
add_executable(${PROJECT_NAME}_std_msgs_sub src/std_msgs_subscriber.cpp)
```

2. Linking the subscriber executable with its dependent catkin package libraries:

```cmake
target_link_libraries(${PROJECT_NAME}_std_msgs_sub
  ${catkin_LIBRARIES}
)
```

3. Installing this new executable alongside the publisher executable:

```cmake
install(TARGETS ${PROJECT_NAME}_std_msgs_pub ${PROJECT_NAME}_std_msgs_sub
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

Create a new file called `std_msgs_subscriber.cpp`, and place it in `adder/src/`.
Copy and paste the following code into this `.cpp` file:

```c++
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
```

Once again, take a moment to read through the comments. If things are still unclear to you, feel free to take a look at the subscriber portion of the [official ROS publisher/subscriber (c++) tutorial](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29).

You may be confused by the parameter type for the subscriber's callback method:

```c++
void SubscriberCB(const std_msgs::Int64MultiArrayConstPtr & msg);
```

Why is there a `const` on the parameter itself, and then a `ConstPtr` at the end of the parameter's type?

* The first `const` is stating that the parameter itself (which is a pointer) cannot be changed.
* `ConstPtr` is stating that you cannot change the data the pointer points to. This means that the pointer can pass the data it points to by reference instead of making a copy of it. This doesn't make much of a difference in our case since our messages are small in size, but this can make a _huge_ difference when passing a message that contains something like point cloud data from a lidar scan.
* _More information about this can be found [here](https://answers.ros.org/question/212857/what-is-constptr/)._

### Testing the Subscriber

In order to generate the executable for our subscriber, we need to re-build our package again.
It's good practice to have a shell dedicated for building workspaces only.
This helps avoid weird bugs that could arise from building a workspace in a shell where you've already sourced it.
So, go back to the shell where you originally built the `adder` package, and re-build it:

```
$ cd ~/ws
$ colcon build
```

Once the package has finished building, We will need to have access to three other shells for the following processes:

1. ROS master
2. Publisher node
3. Subscriber node

We have already discussed how to start processes 1 & 2.
In the third shell, run the subscriber (don't forget to source our `adder` workspace first!):

```
$ rosrun adder adder_std_msgs_sub
```

You should now see some output that indicates our subscriber is adding together messages sent out by our publisher:

```
...

[ INFO] [1590887438.400591716]: Received numbers 5 & 9
[ INFO] [1590887438.403465013]: Sum is 14

[ INFO] [1590887439.400495809]: Received numbers 4 & 6
[ INFO] [1590887439.400610997]: Sum is 10

[ INFO] [1590887440.400511825]: Received numbers 7 & 2
[ INFO] [1590887440.400645092]: Sum is 9

...
```

You have successfully created a ROS publisher and subscriber!

# Next Tutorial

[Using Custom Message Types](./6_custom_msgs.md)