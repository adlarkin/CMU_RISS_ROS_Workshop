# Docker for ROS Melodic

The files in this directory provide a way to use ROS Melodic through a Docker image.
This allows you to avoid installing ROS on your machine directly.

## Usage

### Building the Image

Run the `build_img.bash` script to build the Docker image.
This will create an image similar to the ROS images on [Docker Hub](https://hub.docker.com/_/ros/), but will add a few more features that are needed to view graphical displays like [rqt_graph](https://wiki.ros.org/rqt_graph?distro=melodic).
Running this script will also create a workspace in the home directory of the image, which we will need to create ROS packages later.

This script takes one argument: the name of the image you want to build.

If you wanted to build an image called `ros_tutorials`:

```
$ ./build_img.bash ros_tutorials
```

### Running a Container From the Image

Once you have built the Docker image, you can use it to start a container with a shell that gives you access to ROS.
In order to do this, run the `run_container.bash` script (just like with the `build_img.bash` script, this script enables Docker to be used with GUIs).

This script takes three arguments: the name of the image, the name of the container you're creating, and a path to ROS packages on your system that you'd like to load into the container's workspace.
The third argument (path to ROS packages) is optional; if you do give a path, the files in the directory you state will be loaded into the `src` directory of the container's workspace as a [volume](https://docs.docker.com/storage/volumes/).

Using a Docker volume for your ROS packages means that any changes you make to your packages in the container will persist after the container is destroyed (files created in a Docker container that are not part of a volume only exist for the life of the container).
This allows you to modify the packages in the container for testing, and then save the changes you made on your machine once you're done testing.
Using a volume also lets you use development tools on your machine locally, so there's no need to configure anything inside the container each time you start it!

If you wanted to run a container named `dev_container` using the `ros_tutorials` image from above, you'd enter the following command:

```
$ ./run_container.bash ros_tutorials dev_container
```

If you had several ROS packages on you machine in the `~/packages` directory that you'd like to use in the Docker container's workspace, then you would add this path to the command above:

```
$ ./run_container.bash ros_tutorials dev_container ~/packages
```

### Opening Another Shell in an Already-Existing Container

The `run_container.bash` script will give you a bash session, but sometimes we need to have access to a few terminals for various tasks.
If you'd like to open another terminal that is a part of the container you started with the `run_container.bash` script, you can call the `join.bash` script.

This script takes one argument: the name of the container you want to connect to.

If you wanted to open another terminal that is a part of the container `dev_container`, you' enter the following command:

```
$ ./join.bash dev_container
```