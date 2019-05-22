ARG ROS_DISTRO=kinetic

# ========================================
FROM alpine:latest as cloner

RUN apk add --no-cache git

WORKDIR /repos

COPY . /repos/src/neonavigation
RUN cd /repos/src \
  && git clone --depth=1 https://github.com/at-wat/neonavigation_msgs.git \
  && git clone --depth=1 https://github.com/at-wat/neonavigation_rviz_plugins.git

RUN mkdir -p /repos-manifests/src \
  && find . -name package.xml | xargs -ISRC cp --parents SRC /repos-manifests/

# ========================================
FROM ros:${ROS_DISTRO}-ros-core

RUN apt-get -qq update \
  && apt-get upgrade -y \
  && apt-get install -y --no-install-recommends \
    curl \
    libxml2-utils \
    python-pip \
    sudo \
    wget \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*
RUN pip install gh-pr-comment catkin_lint

RUN rosdep update \
  && mkdir -p /catkin_ws/src \
  && cd /catkin_ws/src \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && catkin_init_workspace

COPY --from=cloner /repos-manifests/src /catkin_ws/src

RUN apt-get -qq update \
  && rosdep install --from-paths /catkin_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

COPY --from=cloner /repos/src /catkin_ws/src
