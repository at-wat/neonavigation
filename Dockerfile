FROM ros:kinetic

RUN apt-get update && \
	apt-get install -y --no-install-recommends sudo ros-kinetic-pcl-ros ros-kinetic-navigation libgtest-dev wget && \
	apt-get clean && \
	rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
	rosdep update && \
	mkdir -p /catkin_ws/src && \
	bash -c "cd /catkin_ws/src && . /opt/ros/${ROS_DISTRO}/setup.bash && catkin_init_workspace && cd .. && catkin_make"

COPY ./ /catkin_ws/src/neonavigation
RUN /catkin_ws/src/neonavigation/.travis.scripts/test.sh



