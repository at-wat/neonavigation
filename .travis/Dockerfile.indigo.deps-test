FROM alpine:3.7 as prebuild

ARG PACKAGE=planner_cspace

COPY . /pkgs
COPY ${PACKAGE}/package.xml /
COPY ${PACKAGE} /target_pkg

RUN cat /package.xml | grep "<depend>" | sed "s/\s*//g" | sed "s/<[^<]*>//g" | while read pkg; do [ -d /pkgs/$pkg ] && echo $pkg || true; done | tee /localdeps \
  && mkdir -p localdeps_ws/src \
  && cat /localdeps | xargs -n1 -i mv /pkgs/{} /localdeps_ws/src/


# =========================
FROM ros:indigo-ros-core

SHELL ["bash", "-c"]

RUN apt-get update -qq \
  && apt-get install -y wget build-essential \
  && apt-get remove -y "ros-${ROS_DISTRO}-*" \
  && apt-get clean && rm -rf /var/lib/apt/list/*

RUN echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list \
  && rosdep update

COPY --from=prebuild /localdeps_ws /localdeps_ws

RUN cd /localdeps_ws \
  && apt-get update -qq \
  && rosdep install -y --from-paths src --ignore-src \
  && apt-get clean && rm -rf /var/lib/apt/lists/* \
  && source /opt/ros/${ROS_DISTRO}/setup.bash \
  && catkin_make_isolated --install --install-space /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && rm -rf devel_isolated build_isolated

WORKDIR /ws

COPY --from=prebuild /localdeps /localdeps
COPY --from=prebuild /target_pkg/package.xml /ws/src/target_pkg/

RUN apt-get update -qq \
  && rosdep install -y --from-paths src --ignore-src --skip-keys "`cat /localdeps | tr "\n" " "`" \
  && apt-get clean && rm -rf /var/lib/apt/lists/*

COPY --from=prebuild /target_pkg /ws/src/target_pkg

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && catkin_make_isolated --install --install-space /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release

