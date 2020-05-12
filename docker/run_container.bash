# TODOs
# 1. replace "ros_tutorials_melodic" with a shell argument "img_name"
# 2. add a shell argument "container_name" that is used to name the spawned container

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
  ros_tutorials_melodic