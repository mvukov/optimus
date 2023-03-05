#!/bin/bash

set -ex

IMAGE=openblas-builder

CONTAINER=${IMAGE}-dummy

docker build ${docker_build_args[@]} -t ${IMAGE} .
docker create --name ${CONTAINER} ${IMAGE}
docker cp ${CONTAINER}:/openblas_ws/install .
docker rm -f ${CONTAINER}
