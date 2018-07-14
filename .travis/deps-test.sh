#!/bin/bash

set -e

cd $(dirname $0)

find ../ -name package.xml | grep -v neonavigation | while read target
do
  dir=$(dirname $target)
  pkg=$(basename $dir)

  echo "# $pkg ($dir)"
  echo
  docker build -t neonavigation-$pkg -f ./Dockerfile.${ROS_DISTRO:-kinetic}.deps-test \
    --build-arg PACKAGE=$pkg ../
  echo
done
