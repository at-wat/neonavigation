#!/bin/bash

set -o errexit

pip install gh-pr-comment

source /opt/ros/${ROS_DISTRO}/setup.bash
source /catkin_ws/devel/setup.bash

set -o verbose

cd /catkin_ws


COVERAGE_OPTION=
if [ x${COVERAGE_TEST} == "xtrue" ]
then
  # Workaround: Ubuntu Trusty and Xenial uses quite old gcc with a bug.
  #   https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65831
  #   Install newer gcc to get correct coverage
  apt-get update -qq
  apt-get install --no-install-recommends software-properties-common -y
  add-apt-repository ppa:ubuntu-toolchain-r/test -y
  apt-get update -qq
  apt-get install --no-install-recommends gcc-5 g++-5 -y
  update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5 --slave /usr/bin/gcov gcov /usr/bin/gcov-5

  gcov -v

  COVERAGE_OPTION=-coverage
fi

sed -i -e "5a set(CMAKE_C_FLAGS \"-Wall -Werror -O2 ${COVERAGE_OPTION}\")" \
  /opt/ros/${ROS_DISTRO}/share/catkin/cmake/toplevel.cmake
sed -i -e "5a set(CMAKE_CXX_FLAGS \"-Wall -Werror -O2 ${COVERAGE_OPTION}\")" \
  /opt/ros/${ROS_DISTRO}/share/catkin/cmake/toplevel.cmake

CM_OPTIONS=""

catkin_make ${CM_OPTIONS} \
  || (gh-pr-comment "FAILED on ${ROS_DISTRO}" '```catkin_make``` failed'; false)
catkin_make tests ${CM_OPTIONS} \
  || (gh-pr-comment "FAILED on ${ROS_DISTRO}" '```catkin_make tests``` failed'; false)
catkin_make run_tests ${CM_OPTIONS} \
  || (gh-pr-comment "FAILED on ${ROS_DISTRO}" '```catkin_make run_tests``` failed'; false)

if [ x${COVERAGE_TEST} == "xtrue" ]
then
  (cd src/neonavigation/; cp -r /catkin_ws/build ./; bash <(curl -s https://codecov.io/bash) -y .codecov.yml)
  exit 0
fi

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

gh-pr-comment "PASSED on ${ROS_DISTRO}" "<details><summary>All tests passed</summary>

$result_text</details>" || true
