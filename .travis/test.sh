#!/bin/bash

set -o errexit

source /opt/ros/${ROS_DISTRO}/setup.bash
cd /catkin_ws

build_number="[[#${TRAVIS_BUILD_NUMBER}](${TRAVIS_BUILD_WEB_URL})]"

md_codeblock='```'

pkgs=$(find . -name package.xml | xargs -n1 dirname)
catkin_lint $pkgs \
  || (gh-pr-comment "${build_number} FAILED on ${ROS_DISTRO}" \
      "<details><summary>catkin_lint failed</summary>

${md_codeblock}
$(catkin_lint $pkgs 2>&1)
${md_codeblock}
</details>"; false)

COVERAGE_OPTION=
if [ x${COVERAGE_TEST} == "xtrue" ]
then
  gcov -v
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

  echo
  echo "Generated gcda files"
  find /catkin_ws/build -name "*.gcda" | xargs -n1 echo "  -"

  # Find and copy renamed gcda files
  echo
  echo "Renamed gcda files"
  find /tmp/gcov/ -name "*.gcda" | sed 's|^/tmp/gcov||' | while read file
  do
    id=$(echo ${file} | cut -d'/' -f2) # /id/path/to/gcda
    gcda=$(echo ${file} | sed 's|^/[^/]*/|/|')
    new_gcda=$(echo ${gcda} | sed "s|/\(\S*\)\.gcda$|/\1.${id}.gcda|")
    gcno=$(echo ${gcda} | sed 's|\.gcda|.gcno|')
    new_gcno=$(echo ${gcda} | sed "s|\.gcda|.${id}.gcno|")
    cp /tmp/gcov/${id}/${gcda} ${new_gcda}
    cp ${gcno} ${new_gcno}
    echo "  - ${new_gcda}"
  done

  cd src/neonavigation/
  cp -r /catkin_ws/build ./

  gcov $(find . -name "*.gcda") -p -c -l > /dev/null

  rm -rf build/neonavigation_rviz_plugins build/neonavigation_msgs
  bash <(curl -s https://codecov.io/bash) \
    -Z \
    -X gcov
fi

if [ catkin_test_results ];
then
  result_text="
${md_codeblock}
$(catkin_test_results --all | grep -v Skipping || true)
${md_codeblock}
"
else
  result_text="
${md_codeblock}
$(catkin_test_results --all | grep -v Skipping || true)
${md_codeblock}
$(find build/test_results/ -name *.xml \
  | xargs -n 1 -- bash -c 'echo; echo \#\#\# $0; echo; echo \\\`\\\`\\\`; xmllint --format $0; echo \\\`\\\`\\\`;')
"
fi
catkin_test_results || (gh-pr-comment "${build_number} FAILED on ${ROS_DISTRO}" "<details><summary>Test failed</summary>

$result_text</details>"; false)

gh-pr-comment "${build_number} PASSED on ${ROS_DISTRO}" "<details><summary>All tests passed</summary>

$result_text</details>" || true
