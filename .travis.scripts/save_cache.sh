#!/bin/bash

set -o errexit
set -o verbose

if [[ ${TRAVIS_BRANCH} == "master" ]] && [[ ${TRAVIS_PULL_REQUEST} == "false" ]];
then
  docker tag docker-neonavigation:latest atwat/ros-neonavigation-cache:latest
  docker login -u ${DOCKER_HUB_USER} -p ${DOCKER_HUB_TOKEN}
	docker push atwat/ros-neonavigation-cache
fi
