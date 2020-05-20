# Commandline Tools

_Previous tutorial: [ROS at a Conceptual Level](./2_ROS_conceptual_level.md)_

Before we get into common usage of ROS commandline tools, you'll need to have gone through the [ROS Docker setup](../docker/README.md).
If you have not done this yet, please take a moment to do so now.

Once you have built the docker image, let's go ahead and start a container.
We don't have any packages to load into the container right now, so you can set the path to anything you'd like. I'll use `~/` for simplicity:

```
$ cd <PATH_TO_REPO>/docker
$ ./run_container.bash ~/
```

The rest of the commands in this tutorial should be run in the Docker container.
If you need to open another shell in the container, remember that you can run `join.bash`:

```
# open a new terminal
$ cd <PATH_TO_REPO>/docker
$ ./join.bash
```

## roscore

As mentioned [previously](./2_ROS_conceptual_level.md#master), we cannot run any ROS programs if the ROS master is not running. The `roscore` command can be used to start the ROS master:

```
$ roscore
```

You should see output similar to the following:

```
... logging to /home/dev/.ros/log/ded22f2c-9a5d-11ea-9820-0242ac110002/roslaunch-fd2881773ac7-63.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://fd2881773ac7:36713/
ros_comm version 1.14.5


SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.5

NODES

auto-starting new master
process[master]: started with pid [73]
ROS_MASTER_URI=http://fd2881773ac7:11311/

setting /run_id to ded22f2c-9a5d-11ea-9820-0242ac110002
process[rosout-1]: started with pid [84]
started core service [/rosout]
```

## rosrun

Now that the ROS master is running, we can start a ROS node (i.e., process).
This can be done with the `rosrun` command.
Leave the ROS master running, open another shell in the Docker container, and enter the following to learn more about `rosrun`:

```
$ rosrun --help
```

As we can see, `rosrun` needs two arguments:
1. A ROS package
2. An executable in this ROS package

Let's run the `turtlesim_node` executable in the `turtlesim` package (this package was installed as a part of the Docker image):

```
$ rosrun turtlesim turtlesim_node
```

You should see a simulator window with a turtle in it.

![The turtlesim simulator](./images/turtlesim.png)

_The turtlesim simulator._

## rqt_graph

The [rqt](https://wiki.ros.org/rqt) ROS metapackage provides a set of GUI tools that are useful for debugging and/or developing ROS programs.
Let's discuss one of the more commonly used tools: `rqt_graph`.

`Rqt_graph` is a great way to visualize how nodes and topics interact with each other.
Make sure that you still have the ROS master and turtlesim simulator running, and run the following in another shell:

```
$ rqt_graph
```

Modify the settings in the GUI to match the image shown below.
We can now see that by running the `turtlesim_node` executable from the `turtlesim` package, we have created a node called `/turtlesim` that subscribes to the topic `/turtle1/cmd_vel` and publishes to the topics `/turtle1/pose` and `/turtle1/color_sensor`.

![The nodes and topics that are created by the turtlesim_node executable](./images/turtlesim_rqt_graph.png)

_The resulting rqt_graph from running the `turtlesim_node` executable.
Note how nodes are ovals, and topics are rectangles._

## rosnode

Although rqt_graph is a great way to visualize how nodes and topics interact, it doesn't provide us with much detailed information for the nodes displayed in the GUI.
If we want to learn more about the current nodes that are a part of a ROS program, we can use `rosnode`.

Let's get a list of the nodes that are currently running. Open another shell (keep running the ROS master, simulator and rqt_graph) and enter the following:

```
$ rosnode list
```

You should see output similar to this:

```
/rosout
/rqt_gui_py_node_725
/turtlesim
```

This means that we have three nodes currently running. 
`/turtlesim` and `/rqt_gui_py_node_725` come from running `turtlesim_node` and `rqt_graph`, respectively.
`/rosout` is a node that always runs during a ROS program, and is used for logging purposes.
More information on `rosout` can be found [here](https://wiki.ros.org/rosout).

If we want to get more information about one of the nodes given by `rosnode list`, we can use `rosnode info`.
Let's learn a little more about the `/turtlesim` node by running the following command:

```
$ rosnode info /turtlesim
```

We are provided with the topics this node publishes/subscribes to (along with the associated message types), as well as the services associated with this node (we will re-visit services later):

```
--------------------------------------------------------------------------------
Node [/turtlesim]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /turtle1/color_sensor [turtlesim/Color]
 * /turtle1/pose [turtlesim/Pose]

Subscriptions: 
 * /turtle1/cmd_vel [unknown type]

Services: 
 * /clear
 * /kill
 * /reset
 * /spawn
 * /turtle1/set_pen
 * /turtle1/teleport_absolute
 * /turtle1/teleport_relative
 * /turtlesim/get_loggers
 * /turtlesim/set_logger_level


contacting node http://fd2881773ac7:34969/ ...
Pid: 668
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (44469 - 172.17.0.2:58030) [15]
    * transport: TCPROS
```

Notice that the message type for the `/turtle1/cmd_vel` topic is unknown.
This is because no messages have been published to this topic yet, so the ROS master does not know what message type is associated with this topic yet.
We will go over how to publish messages to a topic later.

## rostopic

Just like `rosnode`, `rostopic` is a tool that can be used to get detailed information about topics in a ROS program.

Let's get a list of the topics that are currently being published or subscribed to.
In the same shell where we entered the `rosnode` commands, enter the following:

```
$ rostopic list
```

There are 6 topics running in our current setup:

```
/rosout
/rosout_agg
/statistics
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

To get more information about a current topic, we can use `rostopic info`.
Let's get more information about the `/turtle1/pose` topic:

```
$ rostopic info /turtle1/pose
```

We are provided with the topic's message type, publishers, and subscribers:

```
Type: turtlesim/Pose

Publishers: 
 * /turtlesim (http://fd2881773ac7:34969/)

Subscribers: None
```

We can also see if any messages are currently being published to a topic via `rostopic echo`.
Let's see if any messages are being sent to the `/turtle1/color_sensor` topic:

```
$ rostopic echo /turtle1/color_sensor
```

You should see some output that defines the simulator colors (you can stop viewing the most recent messages by entering `ctrl-c`):

```
...

r: 69
g: 86
b: 255
---
r: 69
g: 86
b: 255
---
r: 69
g: 86
b: 255
---
r: 69
g: 86
b: 255
---

...
```

## rosmsg

Using `rosnode` and `rostopic` provide us with the message types associated with each topic.
If we'd like to learn more about a certain message type, we can use `rosmsg`.

When we ran `rostopic info /turtle1/pose` above, we learned that the `/turtle1/cmd_vel` topic is receiving and sending messages of type `turtlesim/Pose`.
We can use `rosmsg show` to learn more about this message type.
In the same shell where we entered the `rostopic` commands, enter the following:

```
$ rosmsg show turtlesim/Pose
```

We're provided with the data fields (or variables) that make up this message:

```
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
```

## rosservice

## rossrv

## rosparam

## roscd & rosls