#!/bin/bash

set -o errexit
set -o verbose

docker load -i ${HOME}/.cache/docker/image.tar || true
