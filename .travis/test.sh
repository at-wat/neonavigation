#!/bin/bash

set -o errexit

source /opt/ros/${ROS_DISTRO}/setup.bash
cd /catkin_ws

build_number="[[#${TRAVIS_BUILD_NUMBER}](${TRAVIS_BUILD_WEB_URL})]"

pkgs=$(find . -name package.xml | xargs -n1 dirname)
catkin_lint $pkgs \
  || (gh-pr-comment "${build_number} FAILED on ${ROS_DISTRO}" \
      "<details><summary>catkin_lint failed</summary>

\`\`\`
$(catkin_lint $pkgs 2>&1)
\`\`\`
</details>"; false)


COVERAGE_OPTION=
if [ x${COVERAGE_TEST} == "xtrue" ]
then
  # Workaround: Ubuntu Xenial uses quite old gcc with a bug.
  #   https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65831
  #   Install newer gcc to get correct coverage
  apt-get update -qq
  if ! apt-cache search "^gcc-5$" | grep gcc-5; then
    echo "gcc-5 not found. enabling ppa:ubuntu-toolchain-r/test"
    apt-get install --no-install-recommends software-properties-common -y
    add-apt-repository ppa:ubuntu-toolchain-r/test -y
    apt-get update -qq
  fi
  apt-get install --no-install-recommends gcc-5 g++-5 -y
  update-alternatives \
    --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 \
    --slave /usr/bin/g++ g++ /usr/bin/g++-5 \
    --slave /usr/bin/gcov gcov /usr/bin/gcov-5

  gcov -v

  # Workaround: Ubuntu Xenial has too old Eigen3 which causes
  #             deprecated warning on gcc-5 with C++11.
  (cd /tmp \
    && git clone -b 3.3.3 --depth=1 https://github.com/eigenteam/eigen-git-mirror.git eigen \
    && mkdir eigen/build \
    && cd eigen/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX=/usr \
    && make install)
  rm -rf /tmp/eigen

  COVERAGE_OPTION=-coverage
fi

sed -i -e "5a set(CMAKE_C_FLAGS \"-Wall -Werror -O2 ${COVERAGE_OPTION}\")" \
  /opt/ros/${ROS_DISTRO}/share/catkin/cmake/toplevel.cmake
sed -i -e "5a set(CMAKE_CXX_FLAGS \"-Wall -Werror -O2 ${COVERAGE_OPTION}\")" \
  /opt/ros/${ROS_DISTRO}/share/catkin/cmake/toplevel.cmake

CM_OPTIONS=${CM_OPTIONS:-}

catkin_make ${CM_OPTIONS} \
  || (gh-pr-comment "${build_number} FAILED on ${ROS_DISTRO}" '```catkin_make``` failed'; false)
catkin_make tests ${CM_OPTIONS} \
  || (gh-pr-comment "${build_number} FAILED on ${ROS_DISTRO}" '```catkin_make tests``` failed'; false)
catkin_make run_tests ${CM_OPTIONS} --make-args -j1 -l1 \
  || (gh-pr-comment "${build_number} FAILED on ${ROS_DISTRO}" '```catkin_make run_tests``` failed'; false)

if [ x${COVERAGE_TEST} == "xtrue" ]
then
  set -o pipefail
  cd src/neonavigation/
  cp -r /catkin_ws/build ./
  rm -rf build/neonavigation_rviz_plugins build/neonavigation_msgs
  bash <(curl -s https://codecov.io/bash) -y .codecov.yml -Z \
    | grep -i -e error -e fail
  exit 0
fi

if [ catkin_test_results ];
then
  result_text="
\`\`\`
$(catkin_test_results --all | grep -v Skipping || true)
\`\`\`
"
else
  result_text="
\`\`\`
$(catkin_test_results --all | grep -v Skipping || true)
\`\`\`
$(find build/test_results/ -name *.xml \
  | xargs -n 1 -- bash -c 'echo; echo \#\#\# $0; echo; echo \\\`\\\`\\\`; xmllint --format $0; echo \\\`\\\`\\\`;')
"
fi
catkin_test_results || (gh-pr-comment "${build_number} FAILED on ${ROS_DISTRO}" "<details><summary>Test failed</summary>

$result_text</details>"; false)

gh-pr-comment "${build_number} PASSED on ${ROS_DISTRO}" "<details><summary>All tests passed</summary>

$result_text</details>" || true
