# based on https://github.com/osrf/docker_images/tree/master/ros/melodic/ubuntu/bionic

FROM ubuntu:bionic

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
  apt-get update && \
  apt-get install -q -y --no-install-recommends tzdata && \
  rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
  dirmngr \
  gnupg2 \
  && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
  build-essential \
  python-rosdep \
  python-rosinstall \
  python-rosinstall-generator \
  python-vcstools \
  python-wstool \
  && rm -rf /var/lib/apt/lists/*

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-melodic-desktop-full \
  ros-melodic-ros-tutorials \
  && rm -rf /var/lib/apt/lists/*

# install other packages (for things like development)
RUN apt-get update && apt-get install -y --no-install-recommends \
  tmux \
  tree \
  vim \
  && rm -rf /var/lib/apt/lists/*

# Add new sudo user (based on https://wiki.ros.org/docker/Tutorials/GUI#The_isolated_way)
# Requires a docker build argument `user_id`
ARG user_id
ENV USERNAME dev
RUN useradd -m $USERNAME && \
  echo "$USERNAME:$USERNAME" | chpasswd && \
  usermod --shell /bin/bash $USERNAME && \
  usermod -aG sudo $USERNAME && \
  echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
  chmod 0440 /etc/sudoers.d/$USERNAME && \
  usermod  --uid ${user_id} $USERNAME && \
  groupmod --gid ${user_id} $USERNAME

USER ${USERNAME}

# bootstrap rosdep
RUN sudo rosdep init && \
  rosdep update

# create a workspace in the user's home directory and add ROS to bashrc
WORKDIR /home/${USERNAME}
RUN mkdir -p ws/src && \
  echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc

CMD [ "/bin/bash" ]