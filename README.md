# ROS_Tutorials
A set of interactive tutorials for learning ROS (Robot Operating System).

These tutorials will cover concepts using ROS Melodic.

## Requirements

### Docker

We will be using [Docker](https://www.docker.com/) so that you do not have to download ROS on your machine directly.
You do not need to have prior knowledge about Docker to use it for these tutorials (although it is helpful), but you will need to install Docker on your machine.
Please visit the tutorial's [docker](./docker/README.md) page and follow the instructions there to get ROS set up on your computer via Docker.

### C++

We will be writing ROS programs in c++, so a basic knowledge of c++ is required.
If you need to brush up on your c++ skills, you can take a look at this [tutorial](https://learncpp.com/).

_CMake knowledge is helpful, but not required._

### Command Line

You should be familiar with basic command line usage, along with things like shell environments.
If you do not have much experience with using the Unix command line, take a look at this [tutorial](http://www.ee.surrey.ac.uk/Teaching/Unix/).

### Host Machine With Unix-Based OS

The Docker images provided in this tutorial rely on the [X Window System](https://en.wikipedia.org/wiki/X_Window_System) to show graphical displays.
Most (if not all) Unix-based OS's should have this installed by default, so it is recommended to go through these tutorials on a Unix-based OS like Ubuntu to guarantee that you can view GUIs through Docker.

If you don't already have a Unix-based OS like Ubuntu on your machine, please take a moment to install one now (Mac users should not have to worry about this).
Windows users can either download an OS like Ubuntu as a virtual machine through something like [Virtualbox](https://www.virtualbox.org/) or dual-boot Ubuntu alongside Windows.

## Tutorials

1. What is ROS?

2. ROS at a Conceptual Level
    * Publisher/Subscriber Model
      * Nodes
      * Topics
      * Messages
    * Services
    * Parameters
    * Master
    * Packages
    * Workspaces
    * Catkin
      * Build System vs Build Tool

3. Commandline Tools
    * roscore
    * rosrun
    * rqt_graph
    * rosnode
    * rostopic
    * rosmsg
    * rosservice
    * rossrv
    * rosparam
    * roscd & rosls

4. Creating Our Own Package
    * catkin_create_pkg
      * CMakeLists.txt vs package.xml

5. Writing Our Own Publisher and Subscriber

6. Using Custom Message Types

7. Roslaunch

## Additional Resources

More ROS tutorials:
* [ROS Wiki](http://wiki.ros.org/) - Overview of ROS concepts/infrastructure, along with the official ROS tutorials
* [A Gentle Introduction to ROS](https://cse.sc.edu/~jokane/agitr/) - Free eBook that covers core ROS concepts
* [ETH Zurich ROS Course](https://rsl.ethz.ch/education-students/lectures/ros.html) - Covers ROS and a few simulation concepts (includes lecture slides and YouTube videos)
* [MIT edX ROS Course](https://www.edx.org/course/hello-real-world-with-ros-robot-operating-system) - Covers basic and more advanced (navigation, manipulation, vision) ROS topics

ROS Community:
* ROS Website: https://www.ros.org/
* ROS Discourse (where lots of ROS announcements are made): https://discourse.ros.org/
* ROS Index (info about ROS packages and repositories, as well as info about ROS 2): https://index.ros.org/
* ROS Answers (useful for ROS debugging/troubleshooting): https://answers.ros.org/questions/

Other ROS Resources:
* Catkin: https://wiki.ros.org/catkin
* ROS Use Patterns and Best Practices: https://wiki.ros.org/ROS/Patterns
* ROS Core Stacks ([rosdistro](https://github.com/ros/rosdistro) is useful): https://github.com/ros
* [packages.ros.org](http://packages.ros.org/)
* URDF: https://wiki.ros.org/urdf
* ROS 2 Design (if you're interested in learning more about ROS 2): http://design.ros2.org/

Simulation:
* Gazebo: http://gazebosim.org/
* Ignition: https://ignitionrobotics.org/home
* SDF: http://sdformat.org/

Docker:
* Docker's official quick start guide: https://docs.docker.com/get-started/
* A more detailed tutorial: https://docker-curriculum.com/
* Docker documentation: https://docs.docker.com/
* NVIDIA Docker (for using Docker containers with NVIDIA GPUs): https://github.com/NVIDIA/nvidia-docker
* Using Docker with ROS: https://wiki.ros.org/docker
* Available Docker images with ROS installed from Docker Hub: https://hub.docker.com/r/osrf/ros
* Official Docker images: https://github.com/docker-library/official-images
