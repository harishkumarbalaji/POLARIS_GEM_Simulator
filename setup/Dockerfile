FROM osrf/ros:noetic-desktop-full

ARG USER_UID=1000
ARG USER_GID=1000
ARG USER=$(whoami)
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y

RUN apt-get install -y \
    git \
    sudo \
    python3-pip \
    build-essential \
    cmake \
    pkg-config \
    libjpeg-dev \
    libtiff5-dev \
    libpng-dev \
    make \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros-control \
    ros-noetic-rosserial \
    ros-noetic-actionlib-tools \
    ros-noetic-ackermann-msgs \
    ros-noetic-geometry2 \
    ros-noetic-hector-gazebo \
    ros-noetic-hector-models \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-velodyne-simulator \
    ros-noetic-septentrio-gnss-driver \
    && rm -rf /var/lib/apt/lists/*

ARG USER
ARG USER_UID
ARG USER_GID

RUN groupadd -g ${USER_GID} ${USER} || groupmod -n ${USER} $(getent group ${USER_GID} | cut -d: -f1)
RUN useradd -l -m -u ${USER_UID} -g ${USER_GID} ${USER} || usermod -l ${USER} -m -u ${USER_UID} -g ${USER_GID} $(getent passwd ${USER_UID} | cut -d: -f1)
RUN echo "${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN chown -R ${USER}:${USER} /home/${USER}

RUN mkdir -p /usr/local/lib/python3.8/dist-packages \
    && chown -R ${USER}:${USER} /usr/local/lib/python3.8/dist-packages/

RUN mkdir -p /catkin_ws/src && chown -R ${USER}:${USER} /catkin_ws

USER ${USER}
RUN sudo chown -R ${USER}:${USER} /opt/ros/noetic

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release"

USER root
RUN chown -R ${USER}:${USER} /home/${USER}
USER ${USER}

# Add ROS setup to user's bashrc
RUN echo 'export PATH=$HOME/.local/bin:$PATH' >> /home/${USER}/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/${USER}/.bashrc

WORKDIR /home/${USER}

# Set container entrypoint
ENTRYPOINT ["/bin/bash"]
