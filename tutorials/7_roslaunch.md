# Roslaunch

_Previous tutorial: [Using Custom Message Types](./6_custom_msgs.md)_

So far, we've had to open a new shell whenever we want to run a ROS process.
This can become irritating rather quickly, especially if you're working on a large project with many nodes.
In this tutorial, we are going to show you how to use a tool called [roslaunch](https://wiki.ros.org/roslaunch) that allows you to run multiple nodes at once.

In order to use `roslaunch`, we must create a `launch` file.
A `launch` file is an `xml` file that defines things like the nodes you'd like to run.
One nice thing about `roslaunch` is that it will automatically start the ROS master if it is not already running.

## Creating Launch Files

Let's create a `launch` file for our `two_int_msg_publisher` node, and then we will discuss how to use it.
Create a directory called `launch` in the `adder` package, and then create a file called `two_int_msg_publisher.launch` in the `launch` directory:

```
$ cd ~/ws/src/adder
$ mkdir launch
$ touch launch/two_int_msg_publisher.launch
```

Now, copy and paste the following code into this `launch` file:

```xml
<launch>
  <node pkg="adder" type="adder_two_ints_msg_pub" name="ints_publisher"/>
</launch>
```

In the same directory, create a file called `two_int_msg_subscriber.launch` and fill it with the following code:

```xml
<launch>
  <node pkg="adder" type="adder_two_ints_msg_sub" name="ints_subscriber" output="screen"/>
</launch>
```

Your `adder` package file structure should now look like this:

```
adder
├── CMakeLists.txt
├── include
│   └── adder
│       └── rand_int_generator.h
├── launch
│   ├── two_int_msg_publisher.launch
│   └── two_int_msg_subsriber.launch
├── package.xml
└── src
    ├── rand_int_generator.cpp
    ├── std_msgs_publisher.cpp
    ├── std_msgs_subscriber.cpp
    ├── two_ints_msg_publisher.cpp
    └── two_ints_msg_subscriber.cpp

4 directories, 10 files
```

Here are a few things to keep in mind about how `launch` files work:

* We start with the `launch` tag.
We need to place any nodes we'd like to run inside the `launch` tag.
* We declare a node we'd like to run by using the `node` tag.
This tag requires 3 arguments:
  * `pkg` - the name of the package that contains the node we'd like to run
  * `type` - the name of the executable for the node
  * `name` - the name we'd like to use for the node.
  Notice that we can use the `name` tag to name our node something different from the name we chose in our `.cpp` file!

You may have noticed that we have an additional argument in our subscriber launch file: `output="screen"`.
By default, `roslaunch` only shows console output for `stderr`.
We'd like to see the output of our `stdout` statements (`ROS_INFO`) in the subscriber, and setting `output=screen` solves this problem.

Let's also make sure we add the following to the end of `adder`'s `CMakeLists.txt`:

```cmake
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
```

Last but not least, let's add a `roslaunch` dependency to `adder`'s `package.xml`:

```xml
<exec_depend>roslaunch</exec_depend>
```

## Using Launch Files

Let's re-build the `adder` package so that we can test out one of these launch files:

```
$ cd ~/ws/
$ colcon build
```

Now, open another shell and try to run the publisher launch file:

```
$ source ~/ws/install/setup.bash

# the general syntax is "roslaunch package_name launch_file_name"
$ roslaunch adder two_int_msg_publisher.launch
```

Notice how the launch file automatically started the ROS master for us!
You won't see any output from this launch file since the `ROS_INFO` statements are sent to a log file instead of the console.

Go ahead and kill the shell running the launch file (`ctrl-c`), and let's discuss how to combine launch files next.

Make another file in the `launch` directory called `add_ints.launch`, and place the following code in it:

```xml
<launch>
  <include file="$(find adder)/launch/two_int_msg_publisher.launch"/>

  <include file="$(find adder)/launch/two_int_msg_subscriber.launch"/>
</launch>
```

Notice how we included the publisher and launch files we just defined above.
We could have just explicitly called the publisher and subscriber nodes in `add_ints.launch`, but it's good practice to have launch files for each individual node so that they can easily be re-used elsewhere.

Now that we have a launch file that calls our custom message publisher and subscriber nodes, let's run it!
Build the packages in our workspace with `colcon`, and then run the following in a separate shell:

```
$ source ~/ws/install/setup.bash
$ roslaunch adder add_ints.launch
```

If you look closely at the initial output from `roslaunch`, you'll notice that the ROS master, publisher, and subscriber were all started by calling this file.
We can also see that our numbers are being published and added as expected!