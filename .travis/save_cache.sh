#!/bin/bash

set -o errexit
set -o verbose

layers=$(echo $@ | xargs -n1 docker history -q | grep -v '<missing>' | sort | uniq)
docker save ${layers} -o ${HOME}/.cache/docker/image.tar
