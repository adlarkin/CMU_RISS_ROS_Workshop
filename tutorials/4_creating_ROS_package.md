# Creating a ROS Package

_Previous tutorial: [Commandline Tools](./3_commandline_tools.md)_

In this tutorial, we will discuss how ROS packages are structured, and go over how to create our own ROS package.

We will be using `catkin` to create a package.
For more information about catkin packages, refer to the official [tutorial](https://wiki.ros.org/catkin/Tutorials/CreatingPackage).

## catkin_create_pkg

We can use the `catkin_create_pkg` command to generate the _"skeleton"_ of our package.
The general usage of this command is as follows:

```
$ catkin_create_pkg package_name [dependencies ...]
```

Let's go ahead and create a package that we will use in future tutorials.
Once we have created this package, we can use it to learn about how packages are structured in ROS.
The package we are creating is called `adder` (this package will add two integers).
We will develop the source code for this package using c++, so this package will depend on [roscpp](https://wiki.ros.org/roscpp), which is a c++ client library for ROS (if you'd like to use Python instead of c++, you can use the [rospy](https://wiki.ros.org/rospy?distro=noetic) client library).

Make a directory on your machine that stores your ROS tutorial packages.
Once this directory has been created, create the `adder` package with a `roscpp` dependency:

```
# on your local machine, make a directory that can hold your ROS packages
$ mkdir ~/packages

# load this directory as a volume in the Docker container
$ cd <PATH_TO_REPO>/docker
$ ./run_dev_container.bash ~/packages

# In the Docker container, create a ROS package inside the mounted volume
$ cd ~/ws/src/
$ catkin_create_pkg adder roscpp
```

_(
Since we are using Docker volumes, you should now see the `adder` package files in your local machine's `packages` directory.
Once you close the Docker container, these ROS package files will still exist on your local machine!
)_

You should see the following output:

```
Created file adder/package.xml
Created file adder/CMakeLists.txt
Created folder adder/include/adder
Created folder adder/src
Successfully created files in /home/dev/ws/src/adder. Please adjust the values in package.xml.
```

Let's take a look at how the file structure is organized for this package:

```
$ tree
.
└── adder
    ├── CMakeLists.txt
    ├── include
    │   └── adder
    ├── package.xml
    └── src

4 directories, 2 files
```

As we can see, `catkin_create_pkg` went ahead and set up the ROS package file structure for us!
Let's go over every directory/file that was created:

* adder
    * The directory that contains all of the source code for the `adder` package
* adder/CMakeLists.txt
    * This file is used to help build the source code in this package
* adder/include/adder
    * The directory for the header source files (`.h` files for corresponding `.cpp` files) in the `adder` package
* adder/package.xml
    * An [xml](https://en.wikipedia.org/wiki/XML) file that provides meta information about the package
* adder/src
    * The directory for the source files (`.cpp` files) in the `adder` package.

## CMakeLists.txt vs package.xml

The `CMakeLists.txt` and `package.xml` files are required for every catkin package.
So, what exactly are these files, and how are they different?

### CmakeLists.txt

The `CmakeLists.txt` file in a catkin package is adopted from the `cmake` build system.
This file helps define things like dependencies and include paths for the **source code** of a given package.
This file also defines where the build and install targets for a package are placed.

Since this `CMakeLists.txt` file is meant for building packages with `catkin`, you will see several features in this file that you wouldn't see in a normal `cmake` project (these are "catkin-specific" features).
I recommend that you take a moment to go through our package's `CMakeLists.txt` file in order to familiarize yourself with the structure.
The file is filled with auto-generated comments, so it's fairly self-explanatory.

If you're still confused about how `catkin` uses `CMakeLists.txt` after reading through the auto-generated comments (or if you'd just like to learn more about how `catkin` uses `CMakeLists.txt`), you can read [the offical catkin CMakelists.txt tutorial](https://wiki.ros.org/catkin/CMakeLists.txt).

### package.xml

The `package.xml` file can be thought of as a _package summary._
This file provides things like the package's description and dependencies.

If you've been following along closely, you may have realized that both the `CMakeLists.txt` and `package.xml` files define dependencies.
So how are these dependencies different?

* `CMakeLists.txt` dependencies are for the **source code** files
* `package.xml` dependencies are for the **whole package**

That raises the next question: _when would you ever have a package dependency that is **not** a source code dependency?_
One example would be if a package had `launch` files that use launch files and/or executables from other packages.
In this scenario, the launch file isn't a part of the source code, so these dependencies aren't a part of the `CMakeLists.txt` (we will go over `launch` files later).

I'd also recommend that you read through our package's `package.xml` file. This file has also been auto-generated with comments that describe how this file is structured, and how this file should be used.

As you read through the `package.xml` file, you may have noticed that there is a `description` tag.
Let's update the description of this package:

```
<description>Adds two ints together via a publisher/subscriber.</description>
```

We can also update the license for our package.
If you aren't sure what license you'd like to use, `MIT` should be fine:

```
<license>MIT</license>
```

Supplemental information about `package.xml` files can be found [here](https://wiki.ros.org/catkin/package.xml).

# Next Tutorial

[Writing Our Own Publisher and Subscriber](./5_publisher_subscriber.md)