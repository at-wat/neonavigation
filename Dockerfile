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
ARG ROS_DISTRO_TARGET=indigo
FROM ros:${ROS_DISTRO_TARGET}-ros-core

RUN apt-get -qq update \
  && apt-get upgrade -y \
  && if [ $ROS_DISTRO == "indigo" ]; then indigo_dep=build-essential; fi \
  && apt-get install -y --no-install-recommends \
    ${indigo_dep:-} \
    curl \
    libxml2-utils \
    python-pip \
    sudo \
    wget \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

RUN rosdep update \
  && mkdir -p /catkin_ws/src \
  && cd /catkin_ws/src \
  && bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash && catkin_init_workspace && cd .. && catkin_make"

COPY --from=cloner /repos-manifests/src /catkin_ws/src

RUN apt-get -qq update \
  && rosdep install --from-paths /catkin_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

COPY --from=cloner /repos/src /catkin_ws/src
