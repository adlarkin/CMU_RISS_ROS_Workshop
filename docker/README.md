# Docker for ROS Melodic

The files in this directory provide a way to use ROS Melodic through a Docker image.
This allows you to avoid installing ROS on your machine directly.

If you have not installed Docker on your computer yet, please take a moment to do so now: https://docs.docker.com/get-docker/

## Usage

### Building the Image

Run the `build_img.bash` script to build the Docker image:

```
$ ./build_img.bash
```

This will create an image similar to the ROS images on [Docker Hub](https://hub.docker.com/_/ros/), but will add a few more features that are needed for viewing graphical displays like [rqt_graph](https://wiki.ros.org/rqt_graph?distro=melodic).
Running this script will also create a workspace in the home directory of the image, which we will need for hosting ROS packages later.

### Running a Container From the Image

Once you have built the Docker image, you can use it to start a container with a shell that gives you access to ROS.
In order to do this, you'll need to run the `run_container.bash` script. Just like the `build_img.bash` script, this script enables Docker to be used with GUIs.

This script takes one argument: the path to the ROS packages on your local filesystem that you'd like to load into the container's workspace.

If you have ROS packages on your machine in the `~/packages` directory that you'd like to use in the Docker container, then you'd enter the following command:

```
$ ./run_container.bash ~/packages
```

The files in the path you give will be loaded into the `~/ws/src` directory of the container as a [volume](https://docs.docker.com/storage/volumes/).
Volumes are a great way to sync files on your local machine with files inside of a Docker container.

Using a Docker volume for your ROS packages means that any changes you make to your packages in the container will persist after the container is destroyed (files created in a Docker container that are not a part of a volume only exist for the life of the container).
Using a volume also lets you use development tools on your machine locally, which removes the need for installing and configuring text editors or IDEs inside Docker.

### Opening Another Shell in an Already-Existing Container

The `run_container.bash` script will give you a bash session, but sometimes we need to have access to multiple shells for various tasks.
If you'd like to open another shell that is a part of the container you started with the `run_container.bash` script, open a new terminal on your machine and call the `join.bash` script:

```
$ ./join.bash
```