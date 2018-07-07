#!/bin/bash

set -o errexit

pip install gh-pr-comment

source /opt/ros/${ROS_DISTRO}/setup.bash
source /catkin_ws/devel/setup.bash

set -o verbose

cd /catkin_ws

C_FLAGS="-Wall -Werror -O2 -coverage"
# Workaround: g++ on Ubuntu Bionic causes false-positive warnings. Disable -Werror on Melodic.
if [ x${ROS_DISTRO} == "xmelodic" ]
then
  C_FLAGS="-Wall -O2 -coverage"
fi

sed -i -e "5a set(CMAKE_C_FLAGS \"${C_FLAGS}\")" \
  /opt/ros/${ROS_DISTRO}/share/catkin/cmake/toplevel.cmake
sed -i -e "5a set(CMAKE_CXX_FLAGS \"${C_FLAGS}\")" \
  /opt/ros/${ROS_DISTRO}/share/catkin/cmake/toplevel.cmake

CM_OPTIONS=""

catkin_make ${CM_OPTIONS} \
  || (gh-pr-comment "FAILED on ${ROS_DISTRO}" '```catkin_make``` failed'; false)
catkin_make tests ${CM_OPTIONS} \
  || (gh-pr-comment "FAILED on ${ROS_DISTRO}" '```catkin_make tests``` failed'; false)
catkin_make run_tests ${CM_OPTIONS} \
  || (gh-pr-comment "FAILED on ${ROS_DISTRO}" '```catkin_make run_tests``` failed'; false)

if [ catkin_test_results ];
then
  result_text="
\`\`\`
`catkin_test_results --all | grep -v Skipping || true`
\`\`\`
"
else
  result_text="
\`\`\`
`catkin_test_results --all | grep -v Skipping || true`
\`\`\`
`find build/test_results/ -name *.xml | xargs -n 1 -- bash -c 'echo; echo \#\#\# $0; echo; echo \\\`\\\`\\\`; xmllint --format $0; echo \\\`\\\`\\\`;'`
"
fi
catkin_test_results || (gh-pr-comment "FAILED on ${ROS_DISTRO}" "<details><summary>Test failed</summary>

$result_text</details>"; false)

# Workaround: Ubuntu Xenial uses quite old gcc with a bug https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65831
# Run coverage check only on Melodic
if [ x${ROS_DISTRO} == "xmelodic" ]
then
  (cd src/neonavigation/; cp -r /catkin_ws/build ./; bash <(curl -s https://codecov.io/bash) -y .codecov.yml)
fi

gh-pr-comment "PASSED on ${ROS_DISTRO}" "<details><summary>All tests passed</summary>

$result_text</details>" || true
