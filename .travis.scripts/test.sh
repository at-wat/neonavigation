#!/bin/bash

set -o errexit
set -o verbose

wget -q -P /tmp https://raw.githubusercontent.com/at-wat/gh-pr-comment/master/gh-pr-comment.sh
source /tmp/gh-pr-comment.sh

source /opt/ros/${ROS_DISTRO}/setup.bash
source /catkin_ws/devel/setup.bash

cd /catkin_ws

apt-get -qq update && \
rosdep install --from-paths src/neonavigation --ignore-src --rosdistro=${ROS_DISTRO} -y && \
apt-get clean && rm -rf /var/lib/apt/lists/*

sync

catkin_make || (gh-pr-comment FAILED '```catkin_make``` failed'; false)
catkin_make tests || (gh-pr-comment FAILED '```catkin_make tests``` failed'; false)
catkin_make run_tests || (gh-pr-comment FAILED '```catkin_make run_tests``` failed'; false)

result_text="
\`\`\`
`catkin_test_results --all || true`
\`\`\`
`find build/test_results/ -name *.xml | xargs -n 1 -- bash -c 'echo; echo \#\#\# $0; echo; echo \\\`\\\`\\\`; xmllint --format $0; echo \\\`\\\`\\\`;'`
"
catkin_test_results || (gh-pr-comment FAILED "Test failed$result_text"; false)

gh-pr-comment PASSED "All tests passed$result_text"

cd ..
rm -rf /catkin_ws || true

