#!/bin/bash

set -o errexit
set -o verbose

docker pull ${DOCKER_CACHE} || true
