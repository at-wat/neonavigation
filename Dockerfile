FROM ros:kinetic

RUN apt-get -qq update \
  && apt-get install -y --no-install-recommends \
    curl \
    libavcodec-dev \
    libavutil-dev \
    libgstreamer1.0 \
    libpcl-dev \
    libqt5gui5 \
    libqtgui4 \
    libyaml-cpp-dev \
    libxml2-utils \
    python-pip \
    python-vtk6 \
    qt5-qmake \
    qtbase5-dev \
    qtbase5-dev-tools \
    qtchooser \
    qtcore4-l10n \
    qttools5-dev \
    qttools5-private-dev \
    sudo \
    tcl-dev \
    tk-dev \
    vtk6 \
    wget \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

RUN rosdep update \
  && mkdir -p /catkin_ws/src \
  && cd /catkin_ws/src \
  && bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash && catkin_init_workspace && cd .. && catkin_make"

COPY ./package.xml /catkin_ws/src/neonavigation/package.xml

RUN apt-get -qq update && \
  rosdep install --from-paths src/neonavigation --ignore-src --rosdistro=${ROS_DISTRO} -y && \
  apt-get clean && rm -rf /var/lib/apt/lists/*

COPY ./ /catkin_ws/src/neonavigation
