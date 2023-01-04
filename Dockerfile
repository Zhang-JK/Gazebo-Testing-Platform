# FROM ubuntu:20.04
FROM nvidia/cudagl:11.4.2-base-ubuntu20.04
LABEL Description="A Dockerfile to setup $RES_NAME:$TAG"

ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace


#
# add the ROS deb repo to the apt sources list
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
          git \
		cmake \
		build-essential \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -


#
# install bootstrap dependencies
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
          libpython3-dev \
          python3-rosdep \
          python3-rosinstall-generator \
          python3-vcstool \
          build-essential && \
    rosdep init && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*


#
# download/build the ROS source
#
RUN mkdir ros_catkin_ws && \
    cd ros_catkin_ws && \
    rosinstall_generator ${ROS_PKG} vision_msgs --rosdistro ${ROS_DISTRO} --deps --tar > ${ROS_DISTRO}-${ROS_PKG}.rosinstall && \
    mkdir src && \
    vcs import --input ${ROS_DISTRO}-${ROS_PKG}.rosinstall ./src && \
    apt-get update && \
    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS_DISTRO} --skip-keys python3-pykdl -y && \
    python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${ROS_ROOT} -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*


#
# setup entrypoint
#
COPY src /root/catkin_ws/src
COPY .catkin_workspace /root/catkin_ws
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /
