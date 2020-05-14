#!/usr/bin/env bash

if [ $# -lt 2 ]
then
  echo "Usage: $0 IMAGE_NAME CONTAINER_NAME [PATH_TO_PACKAGES]"
  exit 1
fi

# load packages into the workspace as a volume if PATH_TO_PACKAGES arg exists
DOCKER_OPTS=
if [ $# -eq 3 ]
then
  CONTAINER_WS_PATH="/home/dev/ws/src/"
  WS_DIR=$3
  echo "Workspace: $WS_DIR -> $CONTAINER_WS_PATH"
  DOCKER_OPTS="$DOCKER_OPTS -v $WS_DIR:$CONTAINER_WS_PATH"
fi

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
  --name $2 \
  $DOCKER_OPTS \
  $1