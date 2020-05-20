#!/usr/bin/env bash

if [ $# -ne 1 ]
then
  echo "Usage: $0 PATH_TO_PACKAGES"
  exit 1
fi

# load packages into the workspace as a volume
DOCKER_OPTS=
CONTAINER_WS_PATH="/home/dev/ws/src/"
WS_DIR=$1
echo "Workspace: $WS_DIR -> $CONTAINER_WS_PATH"
DOCKER_OPTS="$DOCKER_OPTS -v $WS_DIR:$CONTAINER_WS_PATH"

# based on https://wiki.ros.org/docker/Tutorials/GUI#The_isolated_way
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run -it --rm \
  --volume=$XSOCK:$XSOCK:rw \
  --volume=$XAUTH:$XAUTH:rw \
  --env="XAUTHORITY=${XAUTH}" \
  --env="DISPLAY" \
  --name ros_melodic_container \
  $DOCKER_OPTS \
  riss_ros_tutorials_melodic:latest