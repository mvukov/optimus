#!/bin/bash

set -e

OPENBLAS_VERSION=${OPENBLAS_VERSION_MAJOR}.${OPENBLAS_VERSION_MINOR}.${OPENBLAS_VERSION_PATCH}

git clone --branch v${OPENBLAS_VERSION} --depth 1 https://github.com/xianyi/OpenBLAS openblas

mkdir install

cd openblas

OPENBLAS_SUFFIX=nonthreaded
make_options=(
  DYNAMIC_ARCH=0
  TARGET=${OPENBLAS_TARGET}
  CC=${OPENBLAS_CC}
  FC=${OPENBLAS_FC}
  HOSTCC=gcc
  BINARY=64
  INTERFACE=64
  NO_AFFINITY=1
  NO_WARMUP=1
  USE_OPENMP=0
  USE_THREAD=0
  USE_LOCKING=1
  LIBNAMESUFFIX=${OPENBLAS_SUFFIX}
)
make "${make_options[@]}" -j12

install_options=(
  LIBNAMESUFFIX=${OPENBLAS_SUFFIX}
  PREFIX=/openblas_ws/install
)
make "${install_options[@]}" install

set -x

OPENBLAS_TARGET_LOWER=$(echo ${OPENBLAS_TARGET} | tr '[:upper:]' '[:lower:]')
OPENBLAS_SOLIB=libopenblas_${OPENBLAS_SUFFIX}_${OPENBLAS_TARGET_LOWER}-r${OPENBLAS_VERSION}.so
patchelf --set-soname $OPENBLAS_SOLIB /openblas_ws/install/lib/$OPENBLAS_SOLIB
