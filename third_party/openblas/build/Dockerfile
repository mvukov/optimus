FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y \
        build-essential \
        g++ \
        g++-aarch64-linux-gnu \
        gfortran \
        gfortran-aarch64-linux-gnu \
        git \
        patchelf

RUN mkdir /openblas_ws

WORKDIR /openblas_ws

ARG OPENBLAS_VERSION_MAJOR=0
ARG OPENBLAS_VERSION_MINOR=3
ARG OPENBLAS_VERSION_PATCH=21
ARG OPENBLAS_TARGET=HASWELL
ARG OPENBLAS_CC=gcc
ARG OPENBLAS_FC=gfortran

COPY compile_openblas.sh .

RUN ./compile_openblas.sh
