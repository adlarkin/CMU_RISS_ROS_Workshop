# TODOs
# 1. try to make this a multi-staged build that includes openGL )unless I don't need things like rViz?)
# 2. add source /opt/ros/melodic/setup.bash to USERNAME's bashrc

FROM osrf/ros:melodic-desktop-full

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

WORKDIR /home/${USERNAME}

RUN mkdir -p ws/src

CMD [ "/bin/bash" ]